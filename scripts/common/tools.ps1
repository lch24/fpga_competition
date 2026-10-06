# External tools are configured by argument/environment, or discovered on PATH.
function Resolve-ToolDirectory([string]$Directory,[string]$Executable,[string]$Variable) {
    if (!$Directory) {
        $command=Get-Command $Executable -ErrorAction SilentlyContinue
        if (!$command) { throw "Cannot find $Executable. Set $Variable or pass the tool directory explicitly." }
        $Directory=Split-Path $command.Source -Parent
    }
    $Directory=[IO.Path]::GetFullPath($Directory)
    if (!(Test-Path -LiteralPath (Join-Path $Directory $Executable))) { throw "Missing $Executable in $Directory" }
    return $Directory
}

# ModelSim 10.1c may write a FileWatch Tcl message to stderr during exit.
# Preserve the native exit code; each caller additionally checks its PASS marker.
function Invoke-Simulator {
    $executable=$args[0]
    $arguments=@($args | Select-Object -Skip 1)
    $ErrorActionPreference='Continue'
    $output=& $executable @arguments 2>&1
    $code=$LASTEXITCODE
    $output | ForEach-Object { $_.ToString() }
    $global:LASTEXITCODE=$code
}
