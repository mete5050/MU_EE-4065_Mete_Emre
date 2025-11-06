from PIL import Image
import os

# === AYARLAR ===
input_path = r"C:\Users\meteAsus\Desktop\ImgProc\baboon100.png"
output_dir = r"C:\Users\meteAsus\Desktop\ImgProc"
array_name = "g_img"                                # C dizisinin adı
width, height = 100, 100                            # Yeniden boyutlandırma

# =================

# Klasör yoksa oluştur
os.makedirs(output_dir, exist_ok=True)

# Dosya adını çıkar ve çıktı path'ini oluştur
base_name = os.path.splitext(os.path.basename(input_path))[0]
output_path = os.path.join(output_dir, f"{base_name}.h")

# Görseli yükle, gri tonlama yap, yeniden boyutlandır
img = Image.open(input_path).convert("L").resize((width, height))
pixels = list(img.getdata())

# Header dosyasını oluştur
with open(output_path, "w") as f:
    f.write(f"#ifndef {base_name.upper()}_H\n#define {base_name.upper()}_H\n\n")
    f.write("#include <stdint.h>\n\n")
    f.write(f"#define IMG_W {width}\n#define IMG_H {height}\n\n")
    f.write(f"const uint8_t {array_name}[IMG_W * IMG_H] = {{\n")

    for i, val in enumerate(pixels):
        f.write(f"{val}, ")
        if (i + 1) % 16 == 0:
            f.write("\n")

    f.write("};\n\n#endif\n")

print(f"[OK] Header file created at:\n{output_path}")
