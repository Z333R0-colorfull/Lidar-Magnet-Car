import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from scipy import signal
import threading
import time


HOST = "192.168.4.1"   
PORT = 8890
BUFFER_SIZE = 1024
SAMPLE_RATE = 900.0    
FFT_WINDOW_SIZE = 2048  
SLIDING_WIN_SIZE = 21  
SHOW_POINTS = 2048     


raw_queue = deque(maxlen=FFT_WINDOW_SIZE)      
filtered_queue = deque(maxlen=SHOW_POINTS)     
timestamps = deque(maxlen=SHOW_POINTS)


notch_b = None
notch_a = None
filter_z = None      
need_renew_filter = True
lock = threading.Lock()

def design_notch_filter(f0, q=30):
    w0 = f0 / (SAMPLE_RATE / 2)
    b, a = signal.iirnotch(w0, q)
    return b, a

def adaptive_filter_update():
    global notch_b, notch_a, filter_z, need_renew_filter
    with lock:
        if len(raw_queue) < FFT_WINDOW_SIZE:
            return
        data = np.array(raw_queue)
        data_detrend = data - np.mean(data)
        window = np.hanning(len(data))
        fft_vals = np.fft.rfft(data_detrend * window)
        freqs = np.fft.rfftfreq(len(data), d=1/SAMPLE_RATE)
        mag = np.abs(fft_vals)
        valid = (freqs > 1.0) & (freqs < SAMPLE_RATE/2 - 1)
        if not np.any(valid):
            return
        peak_idx = np.argmax(mag[valid])
        peak_freq = freqs[valid][peak_idx]
        print(f"检测到强干扰频率: {peak_freq:.2f} Hz, 幅值: {mag[valid][peak_idx]:.1f}")
        b, a = design_notch_filter(peak_freq, q=30)
        notch_b, notch_a = b, a
        filter_z = signal.lfilter_zi(b, a) * data[0]
        need_renew_filter = False

def apply_filter(new_value):
    global filter_z, need_renew_filter
    if notch_b is None or need_renew_filter:
        return new_value   
    filtered, filter_z = signal.lfilter(notch_b, notch_a, [new_value], zi=filter_z)
    val_notch = filtered[0]
    if not hasattr(apply_filter, "sliding_buf"):
        apply_filter.sliding_buf = deque(maxlen=SLIDING_WIN_SIZE)
    apply_filter.sliding_buf.append(val_notch)
    if len(apply_filter.sliding_buf) == SLIDING_WIN_SIZE:
        return np.mean(apply_filter.sliding_buf)
    else:
        return val_notch   

def receive_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    sockfile = sock.makefile('r')
    print(f"已连接到 {HOST}:{PORT}")
    try:
        for line in sockfile:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) != 4:
                continue
            t_ms = float(parts[0])
            x = float(parts[1])
            y = float(parts[2])   
            z = float(parts[3])
            with lock:
                raw_queue.append(x)
                #raw_queue.append(np.sqrt(x**2+y**2+z**2))
            filtered_x = apply_filter(x)
            #filtered_x = apply_filter(np.sqrt(x**2+y**2+z**2))
            with lock:
                filtered_queue.append(filtered_x)
                timestamps.append(t_ms / 1000.0)   
            if len(raw_queue) >= FFT_WINDOW_SIZE:
                adaptive_filter_update()
    except Exception as e:
        print(f"连接断开: {e}")
    finally:
        sock.close()


fig, ax = plt.subplots()
ax.set_title("Magnetometer X-axis ")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Magnetic Field (uT)")
line_raw, = ax.plot([], [], 'r-', alpha=0.5, label="Raw (before filter)")
line_filt, = ax.plot([], [], 'b-', label="Filtered")
ax.legend()
ax.grid(True)

def animate(frame):
    with lock:
        t_list = list(timestamps)
        raw_list = list(raw_queue)[-len(t_list):]  
        filt_list = list(filtered_queue)
        min_len = min(len(t_list), len(raw_list), len(filt_list))
        t_list = t_list[-min_len:]
        raw_list = raw_list[-min_len:]
        filt_list = filt_list[-min_len:]
    if len(t_list) > 1:
        line_raw.set_data(t_list, raw_list)
        line_filt.set_data(t_list, filt_list)
        ax.relim()
        ax.autoscale_view()
    return line_raw, line_filt

if __name__ == "__main__":
    thread = threading.Thread(target=receive_data, daemon=True)
    thread.start()
    ani = FuncAnimation(fig, animate, interval=50, blit=False)
    plt.show()