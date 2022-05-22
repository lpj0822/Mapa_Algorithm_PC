$threshold = 10
$maxThreshold = 80
$resultPath = "FCWResult.txt"
$totalResultPath = "totalResult.txt"
if(Test-Path $totalResultPath){
    Remove-Item $totalResultPath
}
if(Test-Path $resultPath){
    Remove-Item $resultPath
}
$data = Dir video -filter *.avi
$data | Foreach-Object{
    $videoPath = "video\\" + $_.Name
    echo $videoPath
    ./testSmartEye.exe $videoPath $threshold
}
while ($threshold -le $maxThreshold) {
    if(Test-Path $resultPath){
        Remove-Item $resultPath
    }
    $data = Dir video -filter *.avi
    $data | Foreach-Object{
        $videoPath = "video\\" + $_.Name
        echo $videoPath
        python smartEyeEvalution.py -i $videoPath -f $threshold
    }
    python smartEyeEvalutionTotal.py -i $resultPath
    echo $threshold 
    $threshold+=3
}
python smartEyeRoc.py -i $totalResultPath