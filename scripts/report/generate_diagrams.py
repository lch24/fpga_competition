"""Render the two report diagrams as compact, print-ready PNGs.

Run from any directory: python scripts/report/generate_diagrams.py
Dependency: Pillow. Use --font / --bold-font to select CJK fonts on another OS.
Only the two generated PNGs are written; the Markdown is edited separately.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
INK = "#203247"
MUTED = "#586B7D"
BLUE = "#336A9B"
TEAL = "#1C827D"
GRAY = "#738597"
PURPLE = "#766390"


def choose_font(explicit: Path | None, bold: bool = False) -> Path:
    if explicit:
        if not explicit.is_file():
            raise FileNotFoundError(explicit)
        return explicit
    fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
    candidates = [
        fonts / ("msyhbd.ttc" if bold else "msyh.ttc"),
        Path("/usr/share/fonts/opentype/noto")
        / ("NotoSansCJK-Bold.ttc" if bold else "NotoSansCJK-Regular.ttc"),
        Path("/System/Library/Fonts/PingFang.ttc"),
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError("CJK font not found; pass --font and --bold-font.")


class Diagram:
    """Coordinates use logical pixels; rendering uses 2x resolution by default."""

    def __init__(self, width: int, height: int, font: Path, bold: Path, scale: int):
        self.scale = scale
        self.width, self.height = width, height
        self.image = Image.new("RGB", (width * scale, height * scale), "white")
        self.draw = ImageDraw.Draw(self.image)
        self.font_paths = (font, bold)
        self.fonts: dict[tuple[int, bool], ImageFont.FreeTypeFont] = {}
        self.nodes: dict[str, tuple[int, int, int, int]] = {}
        self.edges: list[list[tuple[int, int]]] = []

    def font(self, size: int, bold: bool = False):
        key = (size, bold)
        if key not in self.fonts:
            self.fonts[key] = ImageFont.truetype(str(self.font_paths[int(bold)]), size * self.scale)
        return self.fonts[key]

    def rect(self, box, fill, outline=None, radius=16, width=2):
        self.draw.rounded_rectangle(
            tuple(round(v * self.scale) for v in box), radius=radius * self.scale,
            fill=fill, outline=outline, width=width * self.scale,
        )

    def text(self, center, text, size=30, color=INK, bold=False, max_width=None):
        font = self.font(size, bold)
        left, top, right, bottom = self.draw.textbbox((0, 0), text, font=font)
        if max_width is not None and right - left > max_width * self.scale:
            raise ValueError(f"Text exceeds allocated width: {text}")
        x = center[0] * self.scale - (left + right) / 2
        y = center[1] * self.scale - (top + bottom) / 2
        if min(x + left, y + top) < 0 or x + right > self.image.width or y + bottom > self.image.height:
            raise ValueError(f"Text outside canvas: {text}")
        self.draw.text((round(x), round(y)), text, font=font, fill=color)

    def heading(self, x, y, text, color, size=29):
        width = self.draw.textlength(text, font=self.font(size, True)) / self.scale
        self.text((x + width / 2, y), text, size, color, True)

    def panel(self, box, title, color, fill, title_x=None):
        self.rect(box, fill, outline="#D9E2E9", radius=20, width=1)
        x, y, _, _ = box
        title_x = x + 42 if title_x is None else title_x
        self.rect((title_x - 20, y + 20, title_x - 14, y + 50), color, radius=3, width=0)
        self.heading(title_x, y + 35, title, color)

    def node(self, name, box, title, subtitle, color=BLUE, fill="white"):
        for other_name, other in self.nodes.items():
            if min(box[2], other[2]) > max(box[0], other[0]) and min(box[3], other[3]) > max(box[1], other[1]):
                raise ValueError(f"Overlapping nodes: {name}, {other_name}")
        self.nodes[name] = box
        self.rect(box, fill, outline=color, radius=14, width=2)
        x, y, right, bottom = box
        middle = (y + bottom) / 2
        self.text(((x + right) / 2, middle - 17), title, 35, INK, True, right - x - 24)
        self.text(((x + right) / 2, middle + 24), subtitle, 25, MUTED, False, right - x - 24)

    def arrowhead(self, point, previous, color, length=13):
        angle = math.atan2(point[1] - previous[1], point[0] - previous[0])
        back = (point[0] - length * math.cos(angle), point[1] - length * math.sin(angle))
        wing = length * 0.48
        points = [point,
                  (back[0] + wing * math.sin(angle), back[1] - wing * math.cos(angle)),
                  (back[0] - wing * math.sin(angle), back[1] + wing * math.cos(angle))]
        self.draw.polygon([(round(x * self.scale), round(y * self.scale)) for x, y in points], fill=color)

    def arrow(self, points, color=BLUE, dashed=False, both=False, width=3):
        self.edges.append(points)
        for start, end in zip(points, points[1:]):
            if start[0] != end[0] and start[1] != end[1]:
                raise ValueError("Use orthogonal routes for readable connections")
            if dashed:
                length = math.dist(start, end)
                for offset in range(0, math.ceil(length), 20):
                    a, b = offset / length, min(offset + 11, length) / length
                    segment = [(start[0] + (end[0] - start[0]) * t,
                                start[1] + (end[1] - start[1]) * t) for t in (a, b)]
                    self.draw.line([(round(x * self.scale), round(y * self.scale)) for x, y in segment],
                                   fill=color, width=width * self.scale)
            else:
                self.draw.line([(x * self.scale, y * self.scale) for x, y in (start, end)],
                               fill=color, width=width * self.scale)
        self.arrowhead(points[-1], points[-2], color)
        if both:
            self.arrowhead(points[0], points[1], color)

    def label(self, center, text, color=MUTED, size=26, fill="white"):
        width = self.draw.textlength(text, font=self.font(size)) / self.scale + 26
        x, y = center
        self.rect((x - width / 2, y - 21, x + width / 2, y + 21), fill, radius=8)
        self.text(center, text, size, color)

    def validate(self):
        # Catch accidental connector routing through an unrelated box. Endpoints
        # may touch borders, but no segment may run inside a node.
        for points in self.edges:
            for a, b in zip(points, points[1:]):
                for name, (x0, y0, x1, y1) in self.nodes.items():
                    if a[0] == b[0]:
                        hit = x0 + 1 < a[0] < x1 - 1 and min(max(a[1], b[1]), y1 - 1) > max(min(a[1], b[1]), y0 + 1)
                    else:
                        hit = y0 + 1 < a[1] < y1 - 1 and min(max(a[0], b[0]), x1 - 1) > max(min(a[0], b[0]), x0 + 1)
                    if hit:
                        raise ValueError(f"Connector crosses node {name}: {a} -> {b}")

    def save(self, path):
        self.validate()
        # ~15 cm intrinsic width, including in exporters that honor PNG DPI.
        dpi = 300 * self.scale
        self.image.save(path, dpi=(dpi, dpi), optimize=True)
        print(f"{path.name}: {self.image.width} x {self.image.height}, {path.stat().st_size:,} bytes")


def system_flow(font, bold, scale):
    d = Diagram(1780, 760, font, bold, scale)
    d.heading(32, 35, "标定计算与实时视频解耦", INK, 37)
    d.panel((350, 95, 1750, 315), "按需执行 · 相机标定与建表", BLUE, "#F2F6FA")
    d.panel((350, 425, 1750, 640), "连续执行 · 视频校正与显示", TEAL, "#F0F7F5")

    d.node("camera", (30, 175, 270, 275), "摄像头采集", "完整图像帧", fill="#F6F8FA", color=GRAY)
    d.node("raw", (30, 500, 270, 600), "原始图像", "DDR 帧缓冲", fill="#F6F8FA", color=GRAY)
    top = [
        ("detect", 390, "角点检测", "多尺度 · 亚像素"),
        ("solve", 740, "参数优化", "内参 / 畸变 / 姿态"),
        ("check", 1090, "结果检查", "保存有效参数"),
        ("build", 1440, "网格建表", "生成坐标映射"),
    ]
    for name, x, title, sub in top:
        d.node(name, (x, 185, x + 280, 285), title, sub)
    bottom = [
        ("display", 390, "HDMI 显示", "原图 / 校正图切换"),
        ("output", 740, "校正图像", "完整帧写回 DDR"),
        ("sample", 1090, "像素插值", "源图四邻域取样"),
        ("remap", 1440, "坐标插值", "读取网格映射"),
    ]
    for name, x, title, sub in bottom:
        d.node(name, (x, 500, x + 280, 600), title, sub, color=TEAL)

    d.arrow([(150, 275), (150, 500)], GRAY)
    d.label((150, 380), "采集写入 DDR", size=25)
    d.arrow([(270, 520), (310, 520), (310, 235), (390, 235)], BLUE)
    for x in (670, 1020, 1370):
        d.arrow([(x, 235), (x + 70, 235)], BLUE)
    d.arrow([(1580, 285), (1580, 500)], BLUE, width=4)
    d.label((1580, 370), "映射跨帧复用", BLUE, 29)
    for x in (1440, 1090, 740):
        d.arrow([(x, 550), (x - 70, 550)], TEAL)
    d.arrow([(150, 600), (150, 705), (1230, 705), (1230, 600)], GRAY)
    d.label((760, 705), "读取源图像素", GRAY, 27)
    d.arrow([(270, 550), (390, 550)], GRAY, dashed=True)
    d.label((550, 674), "虚线：原图预览 / 前后对比", GRAY, 24)
    return d


def shared_compute(font, bold, scale):
    d = Diagram(1780, 820, font, bold, scale)
    d.heading(32, 35, "微程序控制与共享数值计算", INK, 37)
    d.panel((570, 410, 1750, 600), "共享计算通路", TEAL, "#F0F7F5", title_x=1042)

    d.node("stage", (30, 225, 260, 315), "阶段调度", "初值 / LM / 检查")
    d.node("rom", (310, 225, 540, 315), "微程序存储", "程序 ROM")
    d.node("control", (590, 225, 890, 315), "取指与执行控制", "访存 · 运算 · 分支", fill="#EDF4FB")
    d.node("ram", (1020, 65, 1320, 155), "数值工作存储", "矩阵 / 向量 / 中间量")
    d.node("refine", (1410, 65, 1710, 155), "角点精修计算", "共享浮点运算资源", color=PURPLE, fill="#F8F5FA")
    d.node("geometry", (590, 475, 890, 565), "投影与残差服务", "几何模型计算", color=TEAL)
    d.node("arbiter", (1020, 475, 1320, 565), "运算请求仲裁", "记录来源 · 分配后端", color=TEAL)
    d.node("alu", (1410, 475, 1710, 565), "共享浮点后端", "按请求执行数值运算", color=TEAL)
    d.node("return", (1020, 690, 1320, 780), "结果返回", "分发结果 · 更新状态", color=PURPLE, fill="#F8F5FA")

    d.arrow([(260, 270), (310, 270)])
    d.arrow([(540, 270), (590, 270)])
    d.arrow([(740, 225), (740, 110), (1020, 110)], both=True)
    d.label((865, 110), "读 / 写", BLUE, 27)
    d.arrow([(760, 315), (760, 475)], BLUE)
    d.label((760, 380), "调用数值服务", BLUE, 26)
    d.arrow([(890, 270), (960, 270), (960, 495), (1020, 495)], BLUE)
    d.label((960, 350), "基本运算", BLUE, 25)
    d.arrow([(890, 535), (1020, 535)], TEAL)
    d.arrow([(1560, 155), (1560, 365), (1280, 365), (1280, 475)], PURPLE)
    d.label((1425, 365), "精修请求", PURPLE, 27)
    d.arrow([(1320, 520), (1410, 520)], TEAL)
    d.arrow([(1560, 565), (1560, 735), (1320, 735)], PURPLE)
    d.label((1560, 650), "运算完成", PURPLE, 27)
    d.arrow([(1020, 735), (550, 735), (550, 395), (650, 395), (650, 315)], PURPLE)
    d.label((780, 735), "回送结果 · 推进下一条指令", PURPLE, 25)
    return d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "report")
    parser.add_argument("--font", type=Path)
    parser.add_argument("--bold-font", type=Path)
    parser.add_argument("--scale", type=int, default=2, choices=(1, 2, 3))
    args = parser.parse_args()
    regular = choose_font(args.font)
    bold = choose_font(args.bold_font or args.font, bold=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    system_flow(regular, bold, args.scale).save(args.output_dir / "system_flow.png")
    shared_compute(regular, bold, args.scale).save(args.output_dir / "shared_compute.png")


if __name__ == "__main__":
    main()
