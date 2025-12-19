# Anjoman Firmware

[English](#english) | [Farsi](#farsi)

---

<a name="english"></a>
## Project Description

Anjoman Firmware is the core software repository for the "Anjoman" project, a decentralized, leaderless formation control system for differential drive mobile robots.

This project utilizes a hybrid computing architecture to separate low-level real-time control from high-level communication and logic.

### System Architecture

The firmware is designed to run on two interconnected microcontrollers within each robot:

1.  **RP2040 (Raspberry Pi Pico):**
    *   **Role:** Low-level controller.
    *   **Tasks:** Motor driving (DRV8833), Odometry calculation (AS5600 Encoders), IMU data acquisition, and ToF safety sensor management.
    *   **Framework:** Arduino Core (PlatformIO).

2.  **ESP32-S3:**
    *   **Role:** High-level brain and communication node.
    *   **Tasks:** Inter-robot communication (WiFi/ESP-NOW), Ranging (UWB), Formation control algorithms, and Logging.
    *   **Framework:** Arduino Core (PlatformIO).

### Build System
This repository is managed using **PlatformIO**. The directory structure supports multi-environment builds to compile code for both RP2040 and ESP32-S3 from a single source tree.

---

<a name="farsi"></a>
<div dir="rtl">


## توضیحات پروژه

فرمویر انجمن، مخزن اصلی کدهای نرم‌افزاری برای پروژه «انجمن» است. این پروژه یک سامانه کنترل آرایش (Formation Control) غیرمتمرکز و بدون رهبر برای ربات‌های متحرک دیفرانسیلی است.

این سیستم از یک معماری محاسباتی ترکیبی استفاده می‌کند تا پردازش‌های بلادرنگ سطح پایین را از منطق سطح بالا تفکیک کند.

### معماری سیستم

این فرمویر برای اجرا روی دو میکروکنترلر متصل‌به‌هم در هر ربات طراحی شده است:

1.  **RP2040 (رزبری پای پیکو):**
    *   **نقش:** کنترل‌کننده سطح پایین.
    *   **وظایف:** درایو موتورها (DRV8833)، محاسبه موقعیت یا ادومتری (انکودرهای AS5600)، خواندن IMU و سنسور فاصله.
    *   **بستر توسعه:** آردوینو (PlatformIO).

2.  **ESP32-S3:**
    *   **نقش:** پردازشگر منطق اصلی و واحد ارتباطات.
    *   **وظایف:** ارتباط بین ربات‌ها (WiFi/ESP-NOW)، اندازه‌گیری فاصله (UWB)، اجرای الگوریتم‌های کنترل آرایش و ثبت داده‌ها.
    *   **بستر توسعه:** آردوینو (PlatformIO).

### سیستم بیلد
این پروژه توسط **PlatformIO** مدیریت می‌شود. ساختار پوشه‌بندی پروژه به گونه‌ای تنظیم شده است که بتوان کدهای مربوط به هر دو میکروکنترلر RP2040 و ESP32-S3 را در یک محیط واحد توسعه داد و کامپایل کرد.

</div>
