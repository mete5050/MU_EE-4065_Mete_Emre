# save as serial_pgm_to_png.py
# pip install pyserial pillow

import serial, os, time
from datetime import datetime
from PIL import Image

# ==== AYARLAR ====
PORT = "COM8"          # Windows: "COMx"  | macOS: "/dev/tty.usbmodemXXXX" | Linux: "/dev/ttyACM0" vb.
BAUD = 115200
OUTPUT_DIR = "./captures"   # PNG'lerin kaydedileceği klasör
CONTINUOUS = True      # True: sürekli dinle, gelen her resmi kaydet; False: 1 resim al ve çık
TIMEOUT = 5            # seri okuma timeout (s)
# ==================

os.makedirs(OUTPUT_DIR, exist_ok=True)

def _read_line(ser):
    """Seriden bir satır (\\n dahil) okur; yoksa None döner."""
    line = ser.readline()
    return line if line else None

def _parse_pgm_header(ser):
    """
    PGM (P5) header'ını 3 satır okuyup (yorum satırlarını atlayarak) w,h döndürür,
    ayrıca başlığı (bytes) geri verir.
    """
    header_lines = []
    # 1) sihirli değer (P5)
    line = _read_line(ser)
    if not line or not line.startswith(b"P5"):
        return None, None, None  # header yok veya bozuk
    header_lines.append(line)

    # 2) width height satırını bul (yorumlar olabilir)
    while True:
        line = _read_line(ser)
        if line is None:
            return None, None, None
        if line.startswith(b"#"):
            # Yorum satırı: atla
            continue
        # Bu satır w h olmalı
        parts = line.strip().split()
        if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
            w, h = int(parts[0]), int(parts[1])
            header_lines.append(line)
            break
        else:
            # Beklenmeyen satır
            return None, None, None

    # 3) maxval satırı (genelde 255)
    line = _read_line(ser)
    if not line:
        return None, None, None
    # İster 255 olsun ister başka, yazıp geçiyoruz
    header_lines.append(line)

    header_bytes = b"".join(header_lines)
    return w, h, header_bytes

def _read_exact(ser, nbytes):
    """Seriden tam nbytes oku; timeout olursa None."""
    data = bytearray()
    while len(data) < nbytes:
        chunk = ser.read(nbytes - len(data))
        if not chunk:
            # timeout
            return None
        data.extend(chunk)
    return bytes(data)

def receive_one_image(ser, index=None):
    """
    Bir PGM (P5) görüntüyü alır, PNG olarak kaydeder.
    index: dosya numarası (None ise zaman damgası kullanır)
    """
    # Header
    w, h, hdr = _parse_pgm_header(ser)
    if w is None:
        return False, None

    # Piksel verisi (w*h byte)
    pix = _read_exact(ser, w * h)
    if pix is None:
        return False, None

    # (Opsiyonel) STM tarafı "--IMG END--" gibi bir satır gönderiyorsa, onu da tüketelim (zorunlu değil)
    ser.timeout = 0.05
    tail = ser.readline()  # varsa al
    ser.timeout = TIMEOUT

    # PNG kaydet
    img = Image.frombytes("L", (w, h), pix)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    if index is None:
        fname = f"img_{ts}.png"
    else:
        fname = f"img_{index:03d}_{ts}.png"
    out_path = os.path.join(OUTPUT_DIR, fname)
    img.save(out_path, format="PNG")

    return True, out_path

def main():
    print(f"[i] Opening {PORT} @ {BAUD} baud ...")
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        # küçük bir flush
        ser.reset_input_buffer()

        count = 0
        if CONTINUOUS:
            print("[i] Listening continuously. Press Ctrl+C to stop.")
            try:
                while True:
                    ok, path = receive_one_image(ser, index=count)
                    if ok:
                        print(f"[OK] Saved: {path}")
                        count += 1
                    else:
                        print("[!] Failed to parse/receive image (waiting for next P5...)")
                        # P5 gelene kadar devam
            except KeyboardInterrupt:
                print("\n[.] Stopped by user.")
        else:
            ok, path = receive_one_image(ser)
            if ok:
                print(f"[OK] Saved: {path}")
            else:
                print("[!] Failed to receive single image.")

if __name__ == "__main__":
    main()
