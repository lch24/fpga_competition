"""Export Markdown to DOCX while preserving the template's page decoration.

Requires Pillow. Keeps headers, footers, logo and page geometry. Supports this
report's headings, tables, images, inline markup and TeX math subset.
"""
from pathlib import Path
from copy import deepcopy
from xml.etree import ElementTree as E
from zipfile import ZipFile, ZIP_DEFLATED
import argparse
import re
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
NS = {
    "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
    "m": "http://schemas.openxmlformats.org/officeDocument/2006/math",
    "a": "http://schemas.openxmlformats.org/drawingml/2006/main",
    "wp": "http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing",
    "pic": "http://schemas.openxmlformats.org/drawingml/2006/picture",
    "rel": "http://schemas.openxmlformats.org/package/2006/relationships",
}
for prefix, uri in NS.items():
    E.register_namespace(prefix, uri)


def node(name, attrs=None, text=None):
    prefix, local = name.split(":")
    out = E.Element(f"{{{NS[prefix]}}}{local}")
    for key, value in (attrs or {}).items():
        if ":" in key:
            p, k = key.split(":")
            key = f"{{{NS[p]}}}{k}"
        out.set(key, str(value))
    out.text = text
    return out


def add(parent, name, attrs=None, text=None):
    out = node(name, attrs, text)
    parent.append(out)
    return out


def xml(element):
    return E.tostring(element, encoding="unicode")


def math_run(text, style="i"):
    out = node("m:r")
    add(add(out, "m:rPr"), "m:sty", {"m:val": style})
    add(out, "m:t", text=text)
    return out


class Formula:
    """Parse the report's TeX notation into editable Office Math."""
    symbols = {"pi": "π", "theta": "θ", "alpha": "α", "times": "×", "|": "‖"}

    def __init__(self, text):
        self.tokens = re.findall(r"\\[a-zA-Z]+|\\.|[^\s]", text)
        self.pos = 0

    def pop(self):
        out = self.tokens[self.pos]
        self.pos += 1
        return out

    def group(self, style="i"):
        if self.tokens[self.pos] == "{":
            self.pos += 1
            out = self.sequence(style, end="}")
            assert self.pop() == "}"
            return out
        return self.atom(style)

    def limits(self, style):
        out = {}
        while self.pos < len(self.tokens) and self.tokens[self.pos] in ("_", "^"):
            key = self.pop()
            out[key] = self.group(style)
        return out

    def atom(self, style):
        token = self.pop()
        if token == "{":
            out = self.sequence(style, end="}")
            assert self.pop() == "}"
            return out
        if not token.startswith("\\"):
            return [math_run(token, style)]
        command = token[1:]
        if command in self.symbols:
            return [math_run(self.symbols[command], style)]
        if command in ("left", "right"):
            return self.atom(style)
        if command in ("mathbf", "boldsymbol", "mathrm"):
            return self.group({"mathbf": "b", "boldsymbol": "bi", "mathrm": "p"}[command])
        if command == "frac":
            out = node("m:f")
            add(out, "m:num").extend(self.group(style))
            add(out, "m:den").extend(self.group(style))
            return [out]
        if command == "sqrt":
            out = node("m:rad")
            add(add(out, "m:radPr"), "m:degHide", {"m:val": 1})
            add(out, "m:deg")
            add(out, "m:e").extend(self.group(style))
            return [out]
        if command == "hat":
            out = node("m:acc")
            add(add(out, "m:accPr"), "m:chr", {"m:val": "\u0302"})
            add(out, "m:e").extend(self.group(style))
            return [out]
        raise ValueError(f"Unsupported formula command: {token}")

    def sequence(self, style="i", end=None):
        out = []
        while self.pos < len(self.tokens) and self.tokens[self.pos] != end:
            if self.tokens[self.pos] == "\\sum":
                self.pos += 1
                limits = self.limits(style)
                nary = node("m:nary")
                props = add(nary, "m:naryPr")
                add(props, "m:chr", {"m:val": "∑"})
                add(props, "m:limLoc", {"m:val": "undOvr"})
                add(nary, "m:sub").extend(limits.get("_", []))
                add(nary, "m:sup").extend(limits.get("^", []))
                add(nary, "m:e").extend(self.sequence(style, end))
                out.append(nary)
                break
            atoms = self.atom(style)
            limits = self.limits(style)
            if limits:
                tag = "sSubSup" if len(limits) == 2 else "sSub" if "_" in limits else "sSup"
                script = node("m:" + tag)
                add(script, "m:e").extend(atoms)
                for key, child in (("_", "m:sub"), ("^", "m:sup")):
                    if key in limits:
                        add(script, child).extend(limits[key])
                atoms = [script]
            out.extend(atoms)
        return out

    def omml(self):
        out = node("m:oMath")
        out.extend(self.sequence())
        assert self.pos == len(self.tokens)
        return out


class Report:
    inline_pattern = re.compile(r"(\*\*.+?\*\*|`[^`]+`|\$[^$]+\$|\[[^\]]+\]\([^)]+\))")
    image_pattern = re.compile(r"!\[([^\]]*)\]\(([^)]+)\)")

    def __init__(self, source, template):
        self.source = source
        with ZipFile(template) as package:
            self.parts = {n: package.read(n) for n in package.namelist()}
        document = E.fromstring(self.parts["word/document.xml"])
        self.section = deepcopy(document.find("w:body/w:sectPr", NS))
        size = self.section.find("w:pgSz", NS)
        margins = self.section.find("w:pgMar", NS)
        self.width = int(size.get(f"{{{NS['w']}}}w")) - sum(
            int(margins.get(f"{{{NS['w']}}}{key}")) for key in ("left", "right"))
        self.body = node("w:body")
        self.rels = E.fromstring(self.parts["word/_rels/document.xml.rels"])
        self.rid = max(int(r.get("Id")[3:]) for r in self.rels if r.get("Id", "").startswith("rId"))
        self.image_count = self.table_count = self.display_math = self.heading_count = 0
        self.styles = []

    def style(self, name, size, bold=False, heading=None, center=False):
        out = node("w:style", {"w:type": "paragraph", "w:styleId": name})
        add(out, "w:name", {"w:val": name})
        add(out, "w:qFormat")
        props = add(out, "w:pPr")
        add(props, "w:widowControl")
        add(props, "w:snapToGrid", {"w:val": 0})
        add(props, "w:spacing", {"w:before": 180 if heading is not None else 0,
                                "w:after": 100 if heading is not None else 90,
                                "w:line": 310, "w:lineRule": "auto"})
        if heading is not None:
            add(props, "w:keepNext")
            add(props, "w:keepLines")
            if heading >= 0:
                add(props, "w:outlineLvl", {"w:val": heading})
        add(props, "w:jc", {"w:val": "center" if center else "both"})
        run = add(out, "w:rPr")
        add(run, "w:rFonts", {"w:ascii": "Times New Roman", "w:hAnsi": "Times New Roman",
                              "w:eastAsia": "黑体" if bold else "宋体"})
        add(run, "w:sz", {"w:val": int(size * 2)})
        add(run, "w:szCs", {"w:val": int(size * 2)})
        add(run, "w:lang", {"w:val": "en-US", "w:eastAsia": "zh-CN"})
        if bold:
            add(run, "w:b")
        self.styles.append(out)

    def paragraph(self, parent, style="ReportBody", center=False):
        p = add(parent, "w:p")
        props = add(p, "w:pPr")
        add(props, "w:pStyle", {"w:val": style})
        if center:
            add(props, "w:jc", {"w:val": "center"})
        return p

    @staticmethod
    def run(p, text, bold=False, code=False):
        if not text:
            return
        run = add(p, "w:r")
        if bold or code:
            props = add(run, "w:rPr")
            if bold:
                add(props, "w:b")
            if code:
                add(props, "w:rFonts", {"w:ascii": "Consolas", "w:hAnsi": "Consolas"})
                add(props, "w:sz", {"w:val": 19})
        t = add(run, "w:t", text=text)
        t.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")

    def inline(self, p, text, bold=False):
        offset = 0
        for match in self.inline_pattern.finditer(text):
            self.run(p, text[offset:match.start()], bold)
            token = match.group()
            if token.startswith("**"):
                self.inline(p, token[2:-2], True)
            elif token.startswith("`"):
                self.run(p, token[1:-1], bold, code=True)
            elif token.startswith("$"):
                p.append(Formula(token[1:-1]).omml())
            else:
                label, href = re.fullmatch(r"\[([^\]]+)\]\(([^)]+)\)", token).groups()
                self.rid += 1
                add(self.rels, "rel:Relationship", {"Id": f"rId{self.rid}",
                    "Type": NS["r"] + "/hyperlink", "Target": href, "TargetMode": "External"})
                self.run(add(p, "w:hyperlink", {"r:id": f"rId{self.rid}"}), label, bold)
            offset = match.end()
        self.run(p, text[offset:], bold)

    def image(self, parent, path, alt, width):
        path = (self.source.parent / path).resolve()
        if path.suffix.lower() != ".png":
            raise ValueError(f"Expected PNG: {path}")
        with Image.open(path) as bitmap:
            w, h = bitmap.size
        cx = round(min(width * 635, 9.1 * 360000 * w / h))
        cy = round(cx * h / w)
        self.image_count += 1
        self.rid += 1
        part = f"media/report_{self.image_count:02d}.png"
        self.parts["word/" + part] = path.read_bytes()
        add(self.rels, "rel:Relationship", {"Id": f"rId{self.rid}",
            "Type": NS["r"] + "/image", "Target": part})
        p = self.paragraph(parent, "ReportFigure", center=True)
        drawing = add(add(p, "w:r"), "w:drawing")
        inline = add(drawing, "wp:inline", {"distT": 0, "distB": 0, "distL": 0, "distR": 0})
        add(inline, "wp:extent", {"cx": cx, "cy": cy})
        add(inline, "wp:docPr", {"id": self.image_count, "name": path.name, "descr": alt})
        add(add(inline, "wp:cNvGraphicFramePr"), "a:graphicFrameLocks", {"noChangeAspect": 1})
        data = add(add(inline, "a:graphic"), "a:graphicData", {"uri": NS["pic"]})
        picture = add(data, "pic:pic")
        props = add(picture, "pic:nvPicPr")
        add(props, "pic:cNvPr", {"id": self.image_count, "name": path.name})
        add(props, "pic:cNvPicPr")
        fill = add(picture, "pic:blipFill")
        add(fill, "a:blip", {"r:embed": f"rId{self.rid}"})
        add(add(fill, "a:stretch"), "a:fillRect")
        shape = add(picture, "pic:spPr")
        xfrm = add(shape, "a:xfrm")
        add(xfrm, "a:off", {"x": 0, "y": 0})
        add(xfrm, "a:ext", {"cx": cx, "cy": cy})
        add(add(shape, "a:prstGeom", {"prst": "rect"}), "a:avLst")

    def content(self, parent, text, style="ReportBody", width=None):
        width = width or self.width
        parts = self.image_pattern.split(text)
        if len(parts) == 1:
            self.inline(self.paragraph(parent, style), text)
        else:
            if parts[0].strip():
                self.inline(self.paragraph(parent, style), parts[0].strip())
            for i in range(1, len(parts), 3):
                self.image(parent, parts[i + 1], parts[i], width)
                if parts[i + 2].strip():
                    self.inline(self.paragraph(parent, style), parts[i + 2].strip())

    def table(self, lines):
        rows = [[c.strip() for c in line.strip().strip("|").split("|")] for line in lines]
        align = ["right" if c.endswith(":") else "left" for c in rows[1]]
        rows.pop(1)
        columns = len(rows[0])
        assert all(len(r) == columns for r in rows)
        weights = [min(55, max(sum(1 if ord(c) < 256 else 2 for c in row[i])
                              for row in rows)) ** 0.65 for i in range(columns)]
        if any("![" in c for row in rows for c in row):
            weights = [1] * columns
        widths = [round(self.width * w / sum(weights)) for w in weights]
        widths[-1] += self.width - sum(widths)
        table = add(self.body, "w:tbl")
        props = add(table, "w:tblPr")
        add(props, "w:tblW", {"w:w": self.width, "w:type": "dxa"})
        add(props, "w:tblLayout", {"w:type": "fixed"})
        borders = add(props, "w:tblBorders")
        for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
            add(borders, "w:" + edge, {"w:val": "single", "w:sz": 4, "w:color": "BAC4CF"})
        margins = add(props, "w:tblCellMar")
        for edge in ("top", "left", "bottom", "right"):
            add(margins, "w:" + edge, {"w:w": 70, "w:type": "dxa"})
        grid = add(table, "w:tblGrid")
        for width in widths:
            add(grid, "w:gridCol", {"w:w": width})
        for row_index, row in enumerate(rows):
            tr = add(table, "w:tr")
            trprops = add(tr, "w:trPr")
            add(trprops, "w:cantSplit")
            if not row_index:
                add(trprops, "w:tblHeader")
            for col, content in enumerate(row):
                cell = add(tr, "w:tc")
                cellprops = add(cell, "w:tcPr")
                add(cellprops, "w:tcW", {"w:w": widths[col], "w:type": "dxa"})
                add(cellprops, "w:vAlign", {"w:val": "center"})
                if not row_index:
                    add(cellprops, "w:shd", {"w:fill": "EAF0F5", "w:val": "clear"})
                self.content(cell, content, "ReportTable", widths[col] - 160)
                for p in cell.findall("w:p", NS):
                    add(p.find("w:pPr", NS), "w:jc",
                        {"w:val": "center" if not row_index else align[col]})
                    if not row_index:
                        for run in p.findall(".//w:r", NS):
                            rp = run.find("w:rPr", NS)
                            if rp is None:
                                rp = node("w:rPr")
                                run.insert(0, rp)
                            add(rp, "w:b")
        self.table_count += 1
        self.paragraph(self.body, "ReportSpacer")

    def convert(self):
        self.style("ReportBody", 11)
        self.style("ReportTitle", 18, bold=True, heading=-1, center=True)
        self.style("ReportHeading1", 15, bold=True, heading=0)
        self.style("ReportHeading2", 12.5, bold=True, heading=1)
        self.style("ReportHeading3", 11, bold=True, heading=2)
        self.style("ReportTable", 9.5)
        self.style("ReportFigure", 10)
        self.style("ReportSpacer", 2)
        for style in self.styles:
            name = style.get(f"{{{NS['w']}}}styleId")
            if name in ("ReportTable", "ReportFigure", "ReportSpacer"):
                spacing = style.find("w:pPr/w:spacing", NS)
                spacing.set(f"{{{NS['w']}}}after", "50" if name == "ReportFigure" else "20")
                spacing.set(f"{{{NS['w']}}}line", "240")
        lines = self.source.read_text(encoding="utf-8-sig").splitlines()
        index = 0
        while index < len(lines):
            line = lines[index].strip()
            if not line:
                index += 1
                continue
            if line == "$$":
                end = lines.index("$$", index + 1)
                p = self.paragraph(self.body, center=True)
                math = add(p, "m:oMathPara")
                add(add(math, "m:oMathParaPr"), "m:jc", {"m:val": "center"})
                math.append(Formula(" ".join(lines[index + 1:end])).omml())
                self.display_math += 1
                index = end + 1
            elif line.startswith("|"):
                end = index + 1
                while end < len(lines) and lines[end].strip().startswith("|"):
                    end += 1
                self.table(lines[index:end])
                index = end
            elif line.startswith("#"):
                heading, text = line.split(" ", 1)
                style = "ReportTitle" if len(heading) == 1 else f"ReportHeading{min(len(heading)-1, 3)}"
                self.content(self.body, text, style)
                self.heading_count += 1
                index += 1
            elif re.match(r"\d+\.\s", line):
                self.content(self.body, line)
                index += 1
            else:
                end = index + 1
                while (end < len(lines) and lines[end].strip()
                       and not lines[end].startswith(("#", "|", "$$", "!"))
                       and not re.match(r"\d+\.\s", lines[end])):
                    end += 1
                self.content(self.body, " ".join(l.strip() for l in lines[index:end]))
                index = end
        self.body.append(self.section)
        # Preserve original namespace declarations, including mc:Ignorable.
        original = self.parts["word/document.xml"].decode("utf-8")
        self.parts["word/document.xml"] = (original[:original.index("<w:body>")] +
            xml(self.body) + "</w:document>").encode("utf-8")
        original = self.parts["word/styles.xml"].decode("utf-8")
        self.parts["word/styles.xml"] = original.replace("</w:styles>",
            "".join(xml(s) for s in self.styles) + "</w:styles>").encode("utf-8")
        self.parts["word/_rels/document.xml.rels"] = xml(self.rels).encode("utf-8")

    def save(self, output):
        output.parent.mkdir(parents=True, exist_ok=True)
        with ZipFile(output, "w", ZIP_DEFLATED) as package:
            for name, data in self.parts.items():
                package.writestr(name, data)
        with ZipFile(output) as package:
            assert package.testzip() is None
            for name in package.namelist():
                if name.endswith((".xml", ".rels")):
                    E.fromstring(package.read(name))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=ROOT / "report/设计文档.md")
    p.add_argument("--template", type=Path, default=ROOT / "report/模板.docx")
    p.add_argument("--output", type=Path, default=ROOT / "report/设计文档.docx")
    args = p.parse_args()
    if args.output.resolve() in (args.template.resolve(), args.source.resolve()):
        p.error("Output must differ from source and template")
    report = Report(args.source, args.template)
    report.convert()
    report.save(args.output)
    print(f"Saved {args.output}: headings={report.heading_count}, "
          f"tables={report.table_count}, images={report.image_count}, "
          f"display_equations={report.display_math}")


if __name__ == "__main__":
    main()
