# Unity Editor의 공용(Public) 프로필 인바운드 방화벽 규칙을 허용<->차단으로 토글한다.
# 팀원 PC에서 UDP(손 각도/카메라 좌표)를 받아야 할 때만 잠깐 열고, 테스트가 끝나면 다시 닫는 용도.
$ruleName = "Unity 6000.4.0f1 Editor"

# 관리자 권한이 아니면 UAC 승인 창을 띄워 자기 자신을 관리자 권한으로 재실행
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Start-Process powershell -Verb RunAs -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

$rule = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue | Where-Object { $_.Profile -match "Public" }

if (-not $rule) {
    Write-Host "공용(Public) 프로필에서 '$ruleName' 규칙을 찾지 못했습니다. 방화벽에 해당 규칙이 있는지 확인하세요." -ForegroundColor Red
} else {
    $targetAction = if ($rule.Action -eq "Allow") { "Block" } else { "Allow" }
    $setError = $null

    # 규칙 이름이 Domain/Public 프로필에 중복 존재해서 netsh의 name+profile 텍스트 매칭은
    # 선택이 애매해질 수 있다. 이미 고유하게 식별된 $rule(InstanceID 기준) 객체에 직접
    # Set-NetFirewallRule을 걸어서 "어느 규칙을 바꾸는지"의 모호함을 없앤다.
    try {
        $rule | Set-NetFirewallRule -Action $targetAction -ErrorAction Stop
    } catch {
        $setError = $_.Exception.Message
        # CIM 경로가 실패하면 netsh로 한 번 더 시도 (fallback)
        try {
            netsh advfirewall firewall set rule name="$ruleName" profile=public new action=$($targetAction.ToLower())
        } catch {}
    }

    # 방화벽 서비스에 실제로 반영되기까지 지연이 있을 수 있어서, 즉시 한 번만 확인하지 않고
    # 최대 몇 초간 짧게 재시도하며 확인한다.
    $after = $null
    for ($i = 0; $i -lt 10; $i++) {
        Start-Sleep -Milliseconds 500
        $after = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue | Where-Object { $_.Profile -match "Public" }
        if ($after.Action -eq $targetAction) { break }
    }

    if ($after.Action -eq $targetAction) {
        if ($targetAction -eq "Allow") {
            Write-Host "Unity 인바운드(공용 프로필) 상태: 허용(ALLOW)으로 전환했습니다. 테스트 끝나면 다시 실행해서 닫으세요." -ForegroundColor Green
        } else {
            Write-Host "Unity 인바운드(공용 프로필) 상태: 차단(BLOCK)으로 전환했습니다." -ForegroundColor Yellow
        }
    } else {
        Write-Host "전환 실패: $targetAction 으로 바꾸려 했으나 현재 값은 $($after.Action) 입니다." -ForegroundColor Red
        if ($setError) { Write-Host "오류 내용: $setError" -ForegroundColor Red }
    }
}

Read-Host "엔터를 누르면 창이 닫힙니다"
