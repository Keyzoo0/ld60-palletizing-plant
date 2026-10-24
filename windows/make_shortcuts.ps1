# Creates the icons and the clickable shortcuts in D:\PT ELMCECON. Safe to run again.
Add-Type -AssemblyName System.Drawing
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent $here

function New-Icon([string]$path, [string]$text, [string]$hexA, [string]$hexB) {
    $size = 256
    $bmp = New-Object System.Drawing.Bitmap $size, $size
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = 'AntiAlias'; $g.TextRenderingHint = 'AntiAliasGridFit'
    $rect = New-Object System.Drawing.Rectangle 8, 8, ($size - 16), ($size - 16)
    $brush = New-Object System.Drawing.Drawing2D.LinearGradientBrush $rect, ([System.Drawing.ColorTranslator]::FromHtml($hexA)), ([System.Drawing.ColorTranslator]::FromHtml($hexB)), 45
    $gp = New-Object System.Drawing.Drawing2D.GraphicsPath
    $r = 56
    $gp.AddArc($rect.X, $rect.Y, $r, $r, 180, 90); $gp.AddArc($rect.Right - $r, $rect.Y, $r, $r, 270, 90)
    $gp.AddArc($rect.Right - $r, $rect.Bottom - $r, $r, $r, 0, 90); $gp.AddArc($rect.X, $rect.Bottom - $r, $r, $r, 90, 90)
    $gp.CloseFigure(); $g.FillPath($brush, $gp)
    $px = 110
    if ($text.Length -gt 2) { $px = 74 }
    $font = New-Object System.Drawing.Font 'Segoe UI', ([float]$px), ([System.Drawing.FontStyle]::Bold), ([System.Drawing.GraphicsUnit]::Pixel)
    $sf = New-Object System.Drawing.StringFormat; $sf.Alignment = 'Center'; $sf.LineAlignment = 'Center'
    $g.DrawString($text, $font, [System.Drawing.Brushes]::White, (New-Object System.Drawing.RectangleF 0, 0, $size, $size), $sf)
    $g.Dispose()
    # PNG-compressed .ico (supported since Windows Vista)
    $ms = New-Object System.IO.MemoryStream; $bmp.Save($ms, [System.Drawing.Imaging.ImageFormat]::Png); $png = $ms.ToArray()
    $fs = [System.IO.File]::Create($path); $bw = New-Object System.IO.BinaryWriter $fs
    $bw.Write([uint16]0); $bw.Write([uint16]1); $bw.Write([uint16]1)
    $bw.Write([byte]0); $bw.Write([byte]0); $bw.Write([byte]0); $bw.Write([byte]0)
    $bw.Write([uint16]1); $bw.Write([uint16]32); $bw.Write([uint32]$png.Length); $bw.Write([uint32]22)
    $bw.Write($png); $bw.Close(); $bmp.Dispose()
}

function New-Shortcut([string]$lnk, [string]$target, [string]$icon, [string]$desc) {
    $sh = New-Object -ComObject WScript.Shell
    $s = $sh.CreateShortcut($lnk)
    $s.TargetPath = $target; $s.WorkingDirectory = $here; $s.IconLocation = "$icon,0"; $s.Description = $desc
    $s.Save()
}

New-Icon "$here\stop.ico" 'STOP' '#ff6b5e' '#a51d17'
New-Icon "$here\ld60.ico" 'LD' '#2f8fff' '#0a3d8c'
New-Icon "$here\ld60_lite.ico" 'LD' '#6e7d8f' '#2b3542'

New-Shortcut "$root\LD-60 Pabrik Simulasi.lnk" "$here\LD60_Plant_Run.bat" "$here\ld60.ico" 'Jalankan simulasi pabrik LD-60 (Gazebo + dashboard AMR)'
New-Shortcut "$root\LD-60 Pabrik Simulasi (Ringan).lnk" "$here\LD60_Plant_Run_Lite.bat" "$here\ld60_lite.ico" 'Pabrik LD-60 tanpa jendela Gazebo (lebih ringan)'
New-Shortcut "$root\Stop Simulasi.lnk" "$here\Sim_Stop.bat" "$here\stop.ico" 'Hentikan semua simulasi ROS 2'
Get-ChildItem $root -Filter *.lnk | Select-Object Name
