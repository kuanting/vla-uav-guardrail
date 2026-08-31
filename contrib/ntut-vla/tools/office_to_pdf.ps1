<#
.SYNOPSIS
    Export a .pptx or .docx to PDF using the installed Microsoft Office.

.DESCRIPTION
    Both PowerPoint and Word expose a COM automation interface that can save a
    document as PDF with the same fidelity the desktop application produces.
    LibreOffice is not installed on this machine, so this is the available path.

    Two behaviours worth knowing about, both learned the hard way with COM:

    1. A COM export can report success and write nothing. The exit code of the
       automation call is not evidence. This script therefore checks that the
       output file exists, is non-empty, and was written AFTER the script
       started, and fails loudly otherwise.

    2. If an interactive instance of the application is already open, the
       automation call can attach to it and block on a modal dialog. The script
       creates its own instance and closes only what it opened, so an editor the
       user has open is left alone.

.PARAMETER Path
    The .pptx or .docx to convert. The PDF is written alongside it.

.EXAMPLE
    powershell -File tools\office_to_pdf.ps1 -Path docs\VLA-Guardrail-Midterm-Aug2026.pptx
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string[]]$Path
)

$ErrorActionPreference = "Stop"

# PowerPoint's SaveAs format code for PDF, and Word's.
$PP_PDF   = 32
$WD_PDF   = 17

function Convert-One {
    param([string]$InPath)

    $full = (Resolve-Path -LiteralPath $InPath).Path
    $ext  = [System.IO.Path]::GetExtension($full).ToLowerInvariant()
    $pdf  = [System.IO.Path]::ChangeExtension($full, ".pdf")
    $started = Get-Date

    if (Test-Path -LiteralPath $pdf) { Remove-Item -LiteralPath $pdf -Force }

    switch ($ext) {
        ".pptx" {
            Write-Host "==> PowerPoint: $([System.IO.Path]::GetFileName($full))"
            $app = New-Object -ComObject PowerPoint.Application
            try {
                # PowerPoint refuses WithWindow:=$false on some builds, so the
                # presentation is opened normally and closed explicitly below.
                $pres = $app.Presentations.Open($full, $true, $false, $false)
                $pres.SaveAs($pdf, $PP_PDF)
                $pres.Close()
            } finally {
                $app.Quit()
                [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app)
            }
        }
        ".docx" {
            Write-Host "==> Word: $([System.IO.Path]::GetFileName($full))"
            $app = New-Object -ComObject Word.Application
            $app.Visible = $false
            try {
                $doc = $app.Documents.Open($full, $false, $true)
                $doc.SaveAs([ref]$pdf, [ref]$WD_PDF)
                $doc.Close($false)
            } finally {
                $app.Quit()
                [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app)
            }
        }
        ".html" {
            # Word opens HTML natively and keeps headings, tables and styling,
            # so the report is authored once in Markdown, rendered to print HTML,
            # and turned into BOTH required formats here. That avoids adding a
            # Markdown library and python-docx for a job the installed Office
            # already does.
            Write-Host "==> Word (from HTML): $([System.IO.Path]::GetFileName($full))"
            $docx = [System.IO.Path]::ChangeExtension($full, ".docx")
            if (Test-Path -LiteralPath $docx) { Remove-Item -LiteralPath $docx -Force }
            $app = New-Object -ComObject Word.Application
            $app.Visible = $false
            try {
                $doc = $app.Documents.Open($full, $false, $false)
                # 16 = wdFormatDocumentDefault (.docx). Saving to .docx first
                # detaches the document from the HTML source, so the PDF is
                # produced from the same content the .docx carries.
                $doc.SaveAs([ref]$docx, [ref]16)
                $doc.SaveAs([ref]$pdf, [ref]$WD_PDF)
                $doc.Close($false)
            } finally {
                $app.Quit()
                [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app)
            }
            if (-not (Test-Path -LiteralPath $docx)) { throw "no .docx was written for $full" }
            $d = Get-Item -LiteralPath $docx
            Write-Host ("    {0,-52} {1,8:N2} MB" -f $d.Name, ($d.Length / 1MB))
        }
        default { throw "unsupported extension '$ext' (expected .pptx, .docx or .html)" }
    }

    # The check that matters: COM reports success on exports that produced
    # nothing. Existence, size and freshness are the evidence.
    if (-not (Test-Path -LiteralPath $pdf)) {
        throw "no PDF was written for $full"
    }
    $item = Get-Item -LiteralPath $pdf
    if ($item.Length -lt 1024) {
        throw "PDF for $full is only $($item.Length) bytes; treating as a failed export"
    }
    if ($item.LastWriteTime -lt $started) {
        throw "PDF for $full predates this run; a stale file was left behind"
    }

    "{0,-52} {1,8:N2} MB" -f $item.Name, ($item.Length / 1MB)
}

$results = foreach ($p in $Path) { Convert-One -InPath $p }
Write-Host ""
$results | ForEach-Object { Write-Host "    $_" }
Write-Host "==> done."
