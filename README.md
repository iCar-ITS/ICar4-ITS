## Dependensi

Sebelum menjalankan program, pastikan semua dependensi telah terpenuhi. Beberapa dependensi yang dibutuhkan adalah sebagai berikut:

### A. Diinstall dari source:

1. libserialport
2. geographiclib

### B. Diinstall menggunakan `apt`:

1. libpcap-dev

### C. Diinstall menggunakan `pip`:

1. autopep8
2. ipykernel
3. pygame
4. numpy

```bash
pip3 install autopep8 ipykernel pygame numpy
```

### D. Diinstall menggunakan `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

**libserialport** dapat diinstall dari source dengan langkah-langkah berikut:

```bash
git clone git://sigrok.org/libserialport
cd libserialport

./autogen.sh
./configure

make
sudo make install
```

**geographiclib** dapat diinstall dari source dengan langkah-langkah berikut:

```bash
wget https://sourceforge.net/projects/geographiclib/files/distrib-C++/GeographicLib-2.1.2.tar.gz
tar xfpz GeographicLib-2.1.2.tar.gz
cd GeographicLib-2.1.2

mkdir BUILD
cd BUILD

../configure

make
sudo make install
```

> Terkadang library tidak dapat langsung terdeteksi oleh sistem pada saat kompilasi. Untuk mengatasinya, jalankan perintah `sudo ldconfig` pada terminal dan coba kompilasi ulang.

## Direktori Tambahan

Beberapa direktori tambahan dibutuhkan untuk menjalankan program. Direktori tersebut dapat dibuat dengan perintah berikut:

```bash
mkdir -p ~/icar-data/param
mkdir -p ~/icar-data/log
mkdir -p ~/icar-data/route
mkdir -p ~/icar-data/bag
mkdir -p ~/icar-data/rviz
```

## Konfigurasi ICAR 3

Konfigurasi yang digunakan pada ICAR 3 terdiri dari satu file parameter, yaitu `icar.yaml`. File parameter dapat disesuaikan dengan kebutuhan namun pastikan tidak mengubah nama parameter yang sudah ada. File tersebut kemudian disimpan pada `~/icar-data/param/icar.yaml`. Berikut adalah contoh isi file parameter:

```yaml
gps:
  port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10NBO8C-if00-port0
  baud: 115200
  origin:
    lat: -7.277677
    lon: 112.797536

stm32:
  ip: "192.168.50.2"
  port: 9798

odometry:
  to_meter: 0.000455

steering:
  to_rad: -0.000249

pose_estimator:
  cf_ratio_xy: 49
  cf_ratio_theta: 7

icar:
  tf:
    rear_axle: [0.00, 0.00, 0.30, 0.00, 0.00, 0.00]
    front_axle: [2.30, 0.00, 0.30, 0.00, 0.00, 0.00]
    body: [1.15, 0.00, 1.18, 0.00, 0.00, 0.00]
    gps: [2.30, 0.00, 2.05, 0.00, 0.00, 0.00]
    lidar_front: [2.95, 0.00, 0.75, -1.00, -7.00, 179.00]
    lidar_rearright: [-0.61, -0.81, 0.75, 0.00, 0.00, -131.00]
  wheelbase: 2.3
  tyre:
    width: 185
    aspect_ratio: 60
    rim: 15
  body:
    length: 3.5
    width: 1.5
    height: 1.75
```
