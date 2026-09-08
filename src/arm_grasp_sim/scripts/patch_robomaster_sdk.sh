#!/bin/bash
# 修复 DJI robomaster SDK (GitHub master, pip 安装) 在 Python 3.10 / ROS2 驱动下的已知问题。
# SDK 重装后必须重跑: bash patch_robomaster_sdk.sh
SDK=$(python3 -c "import robomaster, os; print(os.path.dirname(robomaster.__file__))")
SP=$(dirname "$SDK")
echo "SDK at: $SDK"

# 1) conn.py: conn_type 用 is 比较字符串, ROS 参数传来的字符串身份不同导致全部分支不匹配
sed -i "s/ is CONNECTION_WIFI_AP/ == CONNECTION_WIFI_AP/; s/ is CONNECTION_WIFI_STA/ == CONNECTION_WIFI_STA/; s/ is CONNECTION_USB_RNDIS/ == CONNECTION_USB_RNDIS/" "$SDK/conn.py"

# 2) config.py: client.py 引用旧名 DEFAULT_CONN_PROTO, 新版只有 DEFAULT_PROTO_TYPE
python3 - "$SDK/config.py" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read()
if "DEFAULT_CONN_PROTO" not in s:
    s=s.replace('DEFAULT_PROTO_TYPE = "udp"', 'DEFAULT_PROTO_TYPE = "udp"\nDEFAULT_CONN_PROTO = DEFAULT_PROTO_TYPE  # compat alias for client.py')
    open(p,"w").write(s)
PY

# 3) libmedia_codec 无 cp310 wheel, 写 no-op stub (视频解码禁用, 抓取不需要)
cat > "$SP/libmedia_codec.py" <<'PYEOF'
"""No-op stub for DJI libmedia_codec (no cp310 wheel). Video/audio decode disabled."""
class _Base:
    def __init__(self, *a, **k): pass
    def decode(self, *a, **k): return []
    def release(self, *a, **k): pass
    def __getattr__(self, name):
        def _noop(*a, **k): return None
        return _noop
class H264Decoder(_Base): pass
class OpusDecoder(_Base): pass
PYEOF

# 依赖: pip install numpy-quaternion av qrcode "numpy<2"
python3 -c "
from robomaster import robot
robot.Robot()
import quaternion, av, qrcode
print('all SDK patches + deps OK')"
