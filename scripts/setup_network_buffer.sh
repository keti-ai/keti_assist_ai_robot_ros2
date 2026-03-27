#!/bin/bash

# 1. 설정값 정의
RMEM_MAX=2147483647
RMEM_DEFAULT=2147483647

echo "보안 및 성능을 위한 네트워크 버퍼 최적화를 시작합니다..."

# 2. /etc/sysctl.conf에 설정 추가 (중복 체크 포함)
modify_sysctl() {
    local key=$1
    local value=$2
    
    # 해당 키가 이미 존재하는지 확인
    if grep -q "^${key}" /etc/sysctl.conf; then
        echo "업데이트 중: ${key} = ${value}"
        sudo sed -i "s/^${key}.*/${key}=${value}/" /etc/sysctl.conf
    else
        echo "새로 추가 중: ${key} = ${value}"
        echo "${key}=${value}" | sudo tee -a /etc/sysctl.conf > /dev/null
    fi
}

# 설정 적용
modify_sysctl "net.core.rmem_max" $RMEM_MAX
modify_sysctl "net.core.rmem_default" $RMEM_DEFAULT

# 3. 변경사항 즉시 적용
echo "------------------------------------------"
echo "시스템에 설정을 즉시 적용합니다..."
sudo sysctl -p

echo "------------------------------------------"
echo "현재 적용된 값 확인:"
sysctl net.core.rmem_max
sysctl net.core.rmem_default

echo "------------------------------------------"
echo "✅ 설정이 완료되었습니다. 이제 재부팅 후에도 유지됩니다."