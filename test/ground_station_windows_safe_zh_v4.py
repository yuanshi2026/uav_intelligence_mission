#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
============================================================
Windows 简易地面站：UDP 起飞 / 降落 / 状态查询版
============================================================

【文件版本】
2026-07-03-v4-配套机载端持续 setpoint 版

【运行位置】
本文件运行在 Windows 电脑上。

【主要功能】
1. 向机载端 UDP 端口发送 TAKEOFF 指令。
2. 向机载端 UDP 端口发送 LAND 指令。
3. 向机载端 UDP 端口发送 STATUS 指令，查询机载脚本状态。
4. 接收机载端回传的 ACK / STATUS / ERROR 文本。

【通信说明】
本地面站发送的是普通 UDP 文本，不是 MAVLink，也不是 ROS 消息。
真正的 OFFBOARD、ARM、起飞、绕圆和降落，都由 Jetson 上的飞行脚本完成。

【常用目标地址】
1. SITL：如果 Windows 要控制 WSL 里的飞行脚本，目标 IP 通常填 WSL 的 IP。
   你可以在 WSL 里执行 hostname -I 查看。
2. 真机：如果 Jetson 的 IP 是 192.168.151.102，就填 192.168.151.102。

【安全提醒】
TAKEOFF 会让无人机进入自动飞行流程，实飞前必须确认现场安全。
LAND 是软件降落，不等于硬件急停，实飞必须保留遥控器接管和 kill 开关。
"""

# ============================================================
# 一、导入依赖库区
# ============================================================

import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText


# ============================================================
# 二、常用参数修改区
# ============================================================

DEFAULT_UAV_IP = "127.0.0.1"          # SITL 常用默认值；真机请改成 192.168.151.102
DEFAULT_REAL_UAV_IP = "192.168.151.102"  # 你之前给出的真机 Jetson IP
DEFAULT_UAV_PORT = 7777               # 必须和机载端脚本监听端口一致
DEFAULT_SEND_REPEAT = 5               # 每次点击按钮重复发送次数；UDP 不保证必达，所以重复几次
DEFAULT_SEND_INTERVAL = 0.08          # 重复发送之间的间隔，单位 s
DEFAULT_RECV_BUFFER = 4096            # 接收机载端回传信息的缓冲区大小


# ============================================================
# 三、地面站主类
# ============================================================

class SimpleGroundStation:
    """
    Windows 简易地面站。

    这个类负责：
    1. 创建窗口界面。
    2. 创建 UDP socket。
    3. 发送 TAKEOFF / LAND / STATUS 指令。
    4. 在后台线程中接收机载端回传。
    """

    # ------------------------------------------------------------
    # 3.1 初始化区
    # ------------------------------------------------------------

    def __init__(self, root):
        """初始化窗口、UDP socket 和接收线程。"""
        self.root = root
        self.root.title("简易无人机地面站 v4")
        self.root.geometry("760x520")

        # UDP socket 使用同一个端口发和收。
        # 不手动绑定端口时，系统会自动分配一个本地端口。
        # 机载端收到指令后，会回复到这个自动分配的端口。
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)

        self.recv_thread_stop = False
        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.recv_thread.start()

        self.build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------
    # 3.2 界面构建区
    # ------------------------------------------------------------

    def build_ui(self):
        """创建地面站窗口界面。"""
        main = tk.Frame(self.root, padx=12, pady=12)
        main.pack(fill=tk.BOTH, expand=True)

        # ------------------------------
        # 目标地址设置区
        # ------------------------------
        addr_frame = tk.LabelFrame(main, text="一、目标无人机地址", padx=10, pady=10)
        addr_frame.pack(fill=tk.X)

        tk.Label(addr_frame, text="无人机/WSL IP：").grid(row=0, column=0, sticky="w")
        self.ip_var = tk.StringVar(value=DEFAULT_UAV_IP)
        self.ip_entry = tk.Entry(addr_frame, textvariable=self.ip_var, width=22)
        self.ip_entry.grid(row=0, column=1, padx=6)

        tk.Label(addr_frame, text="端口：").grid(row=0, column=2, sticky="w")
        self.port_var = tk.StringVar(value=str(DEFAULT_UAV_PORT))
        self.port_entry = tk.Entry(addr_frame, textvariable=self.port_var, width=10)
        self.port_entry.grid(row=0, column=3, padx=6)

        tk.Button(addr_frame, text="填入 SITL 本机 127.0.0.1", command=self.use_local_ip).grid(row=0, column=4, padx=6)
        tk.Button(addr_frame, text="填入真机 192.168.151.102", command=self.use_real_uav_ip).grid(row=0, column=5, padx=6)

        tip = (
            "提示：如果 Windows 控制 WSL，127.0.0.1 不通时，"
            "请在 WSL 里执行 hostname -I，把显示的 IP 填到这里。"
        )
        tk.Label(addr_frame, text=tip, fg="gray").grid(row=1, column=0, columnspan=6, sticky="w", pady=(8, 0))

        # ------------------------------
        # 控制按钮区
        # ------------------------------
        btn_frame = tk.LabelFrame(main, text="二、控制按钮", padx=10, pady=10)
        btn_frame.pack(fill=tk.X, pady=10)

        self.takeoff_btn = tk.Button(
            btn_frame,
            text="发送起飞 TAKEOFF",
            width=22,
            height=2,
            command=self.confirm_takeoff,
            bg="#dff0d8"
        )
        self.takeoff_btn.grid(row=0, column=0, padx=8, pady=4)

        self.land_btn = tk.Button(
            btn_frame,
            text="发送降落 LAND",
            width=22,
            height=2,
            command=self.confirm_land,
            bg="#f2dede"
        )
        self.land_btn.grid(row=0, column=1, padx=8, pady=4)

        self.status_btn = tk.Button(
            btn_frame,
            text="查询状态 STATUS",
            width=22,
            height=2,
            command=self.send_status,
            bg="#d9edf7"
        )
        self.status_btn.grid(row=0, column=2, padx=8, pady=4)

        # ------------------------------
        # 日志区
        # ------------------------------
        log_frame = tk.LabelFrame(main, text="三、通信日志", padx=10, pady=10)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_box = ScrolledText(log_frame, height=18)
        self.log_box.pack(fill=tk.BOTH, expand=True)
        self.log("地面站已启动。")
        self.log("SITL 测试时请先确认机载端脚本已经在 WSL 中运行，并监听 7777 端口。")

    def use_local_ip(self):
        """把目标 IP 设置成 127.0.0.1。"""
        self.ip_var.set("127.0.0.1")
        self.log("目标 IP 已设置为 127.0.0.1。")

    def use_real_uav_ip(self):
        """把目标 IP 设置成真机 Jetson 的 IP。"""
        self.ip_var.set(DEFAULT_REAL_UAV_IP)
        self.log("目标 IP 已设置为真机 %s。" % DEFAULT_REAL_UAV_IP)

    # ------------------------------------------------------------
    # 3.3 地址读取和日志区
    # ------------------------------------------------------------

    def get_target_addr(self):
        """从界面输入框读取目标 IP 和端口。"""
        ip = self.ip_var.get().strip()
        port_text = self.port_var.get().strip()

        if not ip:
            raise ValueError("目标 IP 不能为空")

        try:
            port = int(port_text)
        except ValueError:
            raise ValueError("端口必须是整数")

        if port <= 0 or port > 65535:
            raise ValueError("端口必须在 1~65535 之间")

        return ip, port

    def log(self, text):
        """在日志框里追加一行文本。"""
        line = "[%s] %s\n" % (time.strftime("%H:%M:%S"), text)
        self.log_box.insert(tk.END, line)
        self.log_box.see(tk.END)

    # ------------------------------------------------------------
    # 3.4 指令发送区
    # ------------------------------------------------------------

    def send_command(self, command):
        """
        发送 UDP 文本指令。

        为了降低 UDP 丢包影响，同一个命令会重复发送几次。
        机载端收到重复 TAKEOFF 时会做状态判断，不会重复启动多个任务。
        """
        try:
            ip, port = self.get_target_addr()
        except ValueError as e:
            messagebox.showerror("地址错误", str(e))
            return

        data = command.encode("utf-8")
        addr = (ip, port)

        for i in range(DEFAULT_SEND_REPEAT):
            try:
                self.sock.sendto(data, addr)
                self.log("发送第 %d/%d 次：%s -> %s:%d" % (
                    i + 1,
                    DEFAULT_SEND_REPEAT,
                    command,
                    ip,
                    port
                ))
            except OSError as e:
                self.log("发送失败：%s" % str(e))
                break
            time.sleep(DEFAULT_SEND_INTERVAL)

    def confirm_takeoff(self):
        """起飞按钮：弹窗确认后发送 TAKEOFF。"""
        ok = messagebox.askyesno(
            "确认起飞",
            "确定发送 TAKEOFF 吗？\n\n"
            "请确认：\n"
            "1. 仿真或真机飞行脚本已经启动。\n"
            "2. MAVROS / SLAM / 定位已经正常。\n"
            "3. 实飞时场地和遥控器接管都已确认。"
        )
        if ok:
            self.send_command("TAKEOFF")

    def confirm_land(self):
        """降落按钮：弹窗确认后发送 LAND。"""
        ok = messagebox.askyesno(
            "确认降落",
            "确定发送 LAND 吗？\n\n机载端会先刹停，然后尝试 AUTO.LAND 并上锁。"
        )
        if ok:
            self.send_command("LAND")

    def send_status(self):
        """发送 STATUS 查询指令。"""
        self.send_command("STATUS")

    # ------------------------------------------------------------
    # 3.5 回传接收区
    # ------------------------------------------------------------

    def recv_loop(self):
        """后台线程：接收机载端回传。"""
        while not self.recv_thread_stop:
            try:
                data, addr = self.sock.recvfrom(DEFAULT_RECV_BUFFER)
            except socket.timeout:
                continue
            except OSError:
                break

            text = data.decode("utf-8", errors="ignore")
            msg = "收到 %s:%d 回传：%s" % (addr[0], addr[1], text)

            # tkinter 界面更新必须放回主线程执行。
            self.root.after(0, self.log, msg)

    # ------------------------------------------------------------
    # 3.6 退出清理区
    # ------------------------------------------------------------

    def on_close(self):
        """关闭窗口时清理 UDP socket。"""
        self.recv_thread_stop = True
        try:
            self.sock.close()
        except OSError:
            pass
        self.root.destroy()


# ============================================================
# 四、程序入口区
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = SimpleGroundStation(root)
    root.mainloop()
