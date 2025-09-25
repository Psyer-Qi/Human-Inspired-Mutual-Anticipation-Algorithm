# 遍历batch参数，从1到10
foreach ($type in 1) {
    # 遍历level参数，从0到2
    foreach ($batch in 1..10) {
        # 构建并执行python命令
        $command = "C:\Users\backup\scoop\apps\miniconda3\current\envs\sim\python.exe main.py --method 0 --batch $batch --type $type"
        Write-Host "Executing: $command" # 打印正在执行的命令
        Invoke-Expression $command # 执行命令，等待执行完成
    }
}
