# 设置共享适配器和互联网连接适配器名称，请根据实际情况修改
$PrivateAdapterName = "WLAN"      # 用作热点（私有）的适配器
$PublicAdapterName  = "以太网"      # 提供互联网连接的适配器

# 初始化 ICS 管理对象
$netSharingManager = New-Object -ComObject HNetCfg.HNetShare
$foundPrivate = $false

# 遍历所有适配器，查找私有适配器
foreach ($conn in $netSharingManager.EnumEveryConnection()) {
    $props = $netSharingManager.NetConnectionProps($conn)
    if ($props.Name -eq $PrivateAdapterName) {
        $foundPrivate = $true
        $config = $netSharingManager.INetSharingConfigurationForINetConnection($conn)
        if ($config.SharingEnabled) {
            Write-Host "检测到适配器 '$PrivateAdapterName' 已开启 ICS，共享状态将切换为关闭..."
            # 停止托管网络，再关闭 ICS
            netsh wlan stop hostednetwork
            $config.DisableSharing()
            Write-Host "已关闭 '$PrivateAdapterName' 上的 ICS。"
        }
        else {
            Write-Host "检测到适配器 '$PrivateAdapterName' 未开启 ICS，共享状态将切换为开启..."
            
            # 先关闭所有已开启 ICS 的网络连接
            Write-Host "正在关闭所有已启用 ICS 的网络连接..."
            foreach ($connTemp in $netSharingManager.EnumEveryConnection()) {
                $cfgTemp = $netSharingManager.INetSharingConfigurationForINetConnection($connTemp)
                if ($cfgTemp.SharingEnabled) {
                    $cfgTemp.DisableSharing()
                    $tempProps = $netSharingManager.NetConnectionProps($connTemp)
                    Write-Host "已关闭适配器 '$($tempProps.Name)' 上的 ICS。"
                }
            }
            
            # 对公共适配器开启 ICS（类型 0 表示公共）
            $foundPublic = $false
            foreach ($conn2 in $netSharingManager.EnumEveryConnection()) {
                $props2 = $netSharingManager.NetConnectionProps($conn2)
                if ($props2.Name -eq $PublicAdapterName) {
                    $foundPublic = $true
                    $config2 = $netSharingManager.INetSharingConfigurationForINetConnection($conn2)
                    Write-Host "正在为 '$PublicAdapterName' 开启 ICS 公共共享..."
                    try {
                        $config2.EnableSharing(1)
                    }
                    catch {
                        Write-Host "为 '$PublicAdapterName' 开启 ICS 公共共享失败: $_"
                        break
                    }
                }
            }
            if (-not $foundPublic) {
                Write-Host "未找到公共适配器 '$PublicAdapterName'，请检查名称是否正确。"
                break
            }
            # 对私有适配器开启 ICS 私有共享（类型 1 表示私有）
            Write-Host "正在为 '$PrivateAdapterName' 开启 ICS 私有共享..."
            try {
                $config.EnableSharing(0)
            }
            catch {
                Write-Host "为 '$PrivateAdapterName' 开启 ICS 私有共享失败: $_"
                break
            }
            # 启动托管网络
            netsh wlan start hostednetwork
            Write-Host "已成功开启共享。"
        }
    }
}

if (-not $foundPrivate) {
    Write-Host "未找到私有适配器 '$PrivateAdapterName'，请检查名称是否正确。"
}