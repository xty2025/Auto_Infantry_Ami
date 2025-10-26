from flask import Flask, request, render_template, jsonify, Response
import json
import os
import logging
import requests
import signal
import subprocess
from datetime import datetime
from flask_cors import CORS
import time
import hashlib

# 日志配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("server.log"),
        logging.StreamHandler()
    ]
)

app = Flask(__name__, template_folder='.')
CORS(app)
def get_local_ip():
    """
    获取本机 IP 地址，要求格式为 192.168.137.*
    如果存在多个符合条件的地址，则返回第一个符合的地址；否则返回 None
    """
    import socket

    # 第一种方式：从 gethostbyname_ex 获取所有地址
    try:
        hostname = socket.gethostname()
        ips = socket.gethostbyname_ex(hostname)[2]
        for ip in ips:
            if ip.startswith("192.168.137."):
                return ip
    except Exception as e:
        print(f"获取IP（方式1）失败: {e}")

    # 第二种方式：通过和外部地址建立UDP套接字连接获取本机IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip.startswith("192.168.137."):
            return ip
    except Exception as e:
        print(f"获取IP（方式2）失败: {e}")

    # 如果没有符合条件的IP，则返回 None
    return None

# 动态配置
CONFIG_PATH = os.getenv('CONFIG_PATH', '../../config.json')
local_ip = get_local_ip()
print("find ip",local_ip)
logging.info(f"find ip:{local_ip}")
VIDEO_SOURCE_DEFAULT = f"http://{local_ip}:8081/stream" if local_ip else "http://192.168.137.184:8081/stream"
VIDEO_SOURCE_DEFAULT = "http://127.0.0.1:8081/stream"
VIDEO_SOURCE = os.getenv('VIDEO_SOURCE', VIDEO_SOURCE_DEFAULT)

def load_config():
    """加载配置文件（自动创建缺失文件）"""
    try:
        if not os.path.exists(CONFIG_PATH):
            os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
            with open(CONFIG_PATH, 'w') as f:
                json.dump({"initialized": False}, f)
            logging.info(f"创建默认配置文件: {CONFIG_PATH}")
        
        with open(CONFIG_PATH, 'r') as f:
            return json.load(f)
    except Exception as e:
        logging.error(f"配置操作失败: {str(e)}")
        return {"error": str(e)}

def save_config(data):
    """保存配置文件"""
    try:
        with open(CONFIG_PATH, 'w') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        return True
    except Exception as e:
        logging.error(f"保存失败: {str(e)}")
        return False
    
def kill_aim():
    """杀掉所有aim相关进程"""
    try:
        # use command: ps -ef | grep autoaim | grep -v grep | awk '{print $2}'
        pids = subprocess.check_output(
            "ps -ef | grep Au | grep -v grep | awk '{print $2}'", 
            shell=True
        ).decode().strip().split()
        for pid in pids:
            subprocess.run(['sudo', 'kill', pid])
        return True
    except Exception as e:
        logging.error(f"杀掉aim进程失败: {str(e)}")
        logging.error(f"可能需要sudo visudo配置免密码权限，添加：hustlyrm ALL=(ALL) NOPASSWD: /bin/kill")
        return False
    

@app.route('/')
def index():
    return render_template('editor.html', 
                         server_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

@app.route('/api/config', methods=['GET'])
def get_config():
    config_data = load_config()
    if "error" in config_data:
        return jsonify(config_data), 500
    return jsonify(config_data)

@app.route('/api/config', methods=['POST'])
def update_config():
    try:
        new_config = request.get_json()
        if not new_config:
            return jsonify({"error": "空请求体"}), 400
        
        if save_config(new_config):
            kill_aim()
            return jsonify({"status": "success"})
        return jsonify({"error": "保存失败"}), 500
    except Exception as e:
        logging.error(f"服务器错误: {str(e)}")
        return jsonify({"error": str(e)}), 500

@app.route('/video_feed')
def video_feed():
    def generate():
        try:
            resp = requests.get(VIDEO_SOURCE, stream=True, timeout=10)
            if resp.status_code == 200:
                for chunk in resp.iter_content(chunk_size=1024 * 512):
                    yield chunk
            else:
                logging.warning(f"视频源异常: {resp.status_code}")
        except Exception as e:
            logging.error(f"视频流错误: {str(e)}")
    
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-cache'}
    )
if __name__ == '__main__':
    config = load_config()
    # 先获取 car_name 对应的值
    car_name = config.get("car_name")
    if car_name:
        # 使用 car_name 值获取下一级 JSON 部分
        param = config.get(car_name, {})
        web_debug = param.get("web_debug", True)
    else:
        web_debug = True

    if web_debug is False:
        logging.info("配置中 web_debug 为 false，终止 server 启动")
        exit(0)

    app.run(
        host=os.getenv('FLASK_HOST', '0.0.0.0'),
        port=int(os.getenv('FLASK_PORT', 8080)),
        debug=False,
        threaded=True,
        use_reloader=False
    )