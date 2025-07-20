# unique-navigation_ros2

ROS2 üzerinde çalışan, GPS, Lidar ve IMU verilerini kullanarak otonom navigasyon sağlayan bir Python paketidir. Araç, belirlenen hedefler arasında, engel algılama ve kaçınma özellikleriyle, Ackermann direksiyon modeliyle hareket eder.

---

## Özellikler

- **Çoklu Hedef Noktası:** Araç, önceden tanımlanmış GPS tabanlı hedef konumlar arasında otomatik olarak hareket eder.
- **Lidar Tabanlı Engel Algılama ve Kaçınma:** Lidar verisiyle önündeki engelleri algılar, gerekirse dairesel manevra ile engelden kaçınır.
- **IMU ve Odometri:** Araç konumu ve yönelimi için IMU ve odometri verilerini kullanır.
- **Beklemeli Navigasyon:** Belirli hedeflere ulaşıldığında araç, tanımlı bir süre bekleyerek navigasyona devam eder.
- **Ackermann Direksiyon Modeli:** Gerçekçi araç hareketleri için Ackermann kinematiği ile hız ve yön kontrolü sağlar.

---

## Ana Dosyalar ve Yapı

- **kontrol.py**  
  Tüm navigasyon mantığını içeren ana Python scriptidir.  
  Temel sınıf: `AckermannSteering`
  - Hedef listesi ve oryantasyonlar
  - Lidar ve odometri abonelikleri
  - Engel algılama, kaçınma algoritması, hedefe ilerleme
  - Belirli hedeflerde bekleme ve navigasyona devam etme
  - Hareket komutlarını cmd_vel üzerinden yayınlama

---

## Kurulum

1. ROS2 ve Python 3.8+ kurulu olmalıdır.
2. Bağımlılıklar:
    ```bash
    pip install rclpy geometry_msgs nav_msgs sensor_msgs
    ```
3. Python dosyasını ROS2 workspace'inizde uygun bir pakete ekleyin.

---

## Kullanım

Terminalde aşağıdaki gibi başlatabilirsiniz:
```bash
ros2 run <paket_adı> kontrol.py
```
veya doğrudan:
```bash
python3 kontrol.py
```

> Not: Lidar (`/scan`), Odometri (`/odom`) ve IMU topic'leri uygun şekilde eşlenmiş olmalıdır.

---

## Temel Akış ve Algoritmalar

- Hedeflere sırasıyla gidilir.
- Engel algılandığında 10 saniye boyunca dairesel hareketle kaçınma uygulanır.
- Engel yoksa hedefe yönelme ve pozisyon/oryantasyon kontrolü yapılır.
- Bazı hedeflerde (örn. 10. ve 22. nokta) araç 30 saniye bekler ve sonra devam eder.
- Tüm hedeflere ulaşıldığında "All goals reached!" mesajı yayınlanır.

---

## Koddan Örnekler

**Engel Algılama:**
```python
def is_obstacle_detected(self):
    front_ranges = self.laser_data[len(self.laser_data)//2 - 15 : len(self.laser_data)//2 + 15]
    min_distance = min(front_ranges)
    return min_distance < 8.0
```

**Hedefe Hareket:**
```python
def move_to_goal(self):
    # ... mesafe ve açı hesabı
    if distance > 1.75:
        cmd_msg.linear.x = 2.0
        cmd_msg.angular.z = 0.2 * angular_error
        self.publisher_.publish(cmd_msg)
    else:
        self.stop_robot()
        # hedefe ulaştıktan sonra bekleme veya sıradaki hedefe geçiş
```

---

## Katkı

Pull request ve issue açarak katkıda bulunabilirsiniz.

---

## İletişim

Proje sahibi: [Mertsr](https://github.com/Mertsr)

---
