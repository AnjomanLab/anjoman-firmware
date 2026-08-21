# Anjoman Firmware

[English](#english) | [فارسی](#فارسی)

---

<a name="english"></a>

## English

### Overview

**Anjoman** is an experimental firmware project for a decentralized swarm of autonomous differential-drive mobile robots.

The project aims to develop a group of heterogeneous robots capable of operating cooperatively without relying on a central controller or fixed external localization infrastructure. Each robot is intended to operate as an autonomous peer while exchanging information with neighboring robots.

The project is inspired by the concept of **Anjoman** (gathering), representing a group of autonomous agents that coordinate through cooperation rather than centralized command.

### System Goals

The firmware is being developed around the following objectives:

- Decentralized multi-robot coordination
- Relative localization using inter-robot UWB ranging
- Sensor fusion using UWB, IMU, and wheel odometry
- Autonomous differential-drive motion control
- Formation control and cooperative navigation
- Collision avoidance
- Peer-to-peer communication
- Real-time and deterministic control execution
- Support for heterogeneous robot hardware

### Robot Platform

The current swarm consists of four differential-drive mobile robots based on ESP32-S3 microcontrollers.

Although the robots share a common software architecture, their mechanical characteristics are not completely identical. Differences in wheel dimensions, drivetrain parameters, and physical sensor placement are therefore considered by the firmware architecture.

### Localization

Anjoman does not rely on fixed UWB anchors.

Instead, UWB transceivers installed on the robots are used for **peer-to-peer ranging**. The ranging measurements are intended to be combined with onboard inertial sensing and wheel odometry to estimate the state of each robot.

The long-term localization architecture is therefore based on relative measurements and distributed state estimation rather than conventional anchor-based RTLS.

### Control

The control architecture is designed around multiple real-time layers.

Low-level control is responsible for sensing, motor control, and maintaining the real-time behavior of each robot. Higher-level components handle communication, state estimation, coordination, and formation behavior.

The final system is intended to allow the robots to operate autonomously while maintaining a coherent formation and avoiding collisions.

### Development Status

The project is currently in the hardware bring-up, sensor characterization, and system identification stage.

The following areas have been investigated or implemented to various degrees:

- Multi-robot build configuration
- Hardware bring-up
- UWB ranging and characterization
- Sensor data acquisition
- Peer-to-peer communication
- Robot-specific configuration
- Initial motor and sensor system identification

The major upcoming stages include:

- Closed-loop wheel velocity control
- Sensor calibration
- State estimation and sensor fusion
- Robust multi-robot UWB scheduling
- Formation control
- Cooperative motion and collision avoidance

### Repository Structure

```text
anjoman-firmware/
├── include/        # Shared configuration and interface definitions
├── src/            # Main firmware sources
├── lib/            # Project-specific modules
├── examples/       # Experimental and characterization code
├── scripts/        # Analysis and development utilities
└── test/           # Tests
````

### Development Environment

The project is developed primarily on Linux using a command-line workflow.

Main tools and technologies include:

* C++
* Arduino framework
* ESP32-S3
* PlatformIO
* FreeRTOS
* UWB / DW1000
* IMU and wheel encoders
* ESP-NOW
* Git

### Project Philosophy

Anjoman is developed as a research-oriented robotics system rather than a collection of independent sensor demonstrations.

The emphasis is on:

* Decentralization
* Deterministic real-time behavior
* Hardware-aware software architecture
* Measurable system performance
* Reproducible experiments
* Modular development
* Integration of perception, estimation, control, and communication

---

<a name="فارسی"></a>

## فارسی

### معرفی

**انجمن** یک پروژه‌ی آزمایشی توسعه‌ی Firmware برای یک گروه از ربات‌های متحرک خودران با مکانیزم حرکتی Differential Drive است.

هدف پروژه، توسعه‌ی گروهی از ربات‌های ناهمگن است که بتوانند بدون اتکا به یک کنترل‌کننده‌ی مرکزی یا زیرساخت ثابت خارجی برای مکان‌یابی، به‌صورت جمعی و هماهنگ فعالیت کنند. هر ربات به‌عنوان یک عامل مستقل در نظر گرفته می‌شود که با سایر ربات‌ها ارتباط برقرار کرده و در تصمیم‌گیری جمعی مشارکت می‌کند.

نام پروژه از مفهوم **انجمن** به معنای گردهمایی و همکاری مجموعه‌ای از عامل‌های مستقل الهام گرفته شده است.

### اهداف سیستم

Firmware انجمن با اهداف زیر توسعه می‌یابد:

* هماهنگی غیرمتمرکز چند ربات
* مکان‌یابی نسبی با استفاده از فاصله‌سنجی UWB بین ربات‌ها
* ترکیب اطلاعات UWB، IMU و انکودرهای چرخ
* کنترل خودران ربات‌های Differential Drive
* کنترل تشکیل آرایش و حرکت جمعی
* اجتناب از برخورد
* ارتباط Peer-to-Peer بین ربات‌ها
* اجرای کنترل بلادرنگ و قابل پیش‌بینی
* پشتیبانی از سخت‌افزار ناهمگن ربات‌ها

### پلتفرم ربات‌ها

گروه فعلی شامل چهار ربات متحرک Differential Drive مبتنی بر میکروکنترلرهای ESP32-S3 است.

اگرچه ربات‌ها از یک معماری نرم‌افزاری مشترک استفاده می‌کنند، مشخصات مکانیکی آن‌ها کاملاً یکسان نیست. تفاوت در ابعاد چرخ‌ها، مشخصات انتقال قدرت و محل قرارگیری برخی حسگرها در معماری Firmware در نظر گرفته شده است.

### مکان‌یابی

انجمن از Anchorهای ثابت UWB استفاده نمی‌کند.

در عوض، فرستنده‌گیرنده‌های UWB نصب‌شده روی ربات‌ها برای **اندازه‌گیری فاصله‌ی بین ربات‌ها** استفاده می‌شوند. این اندازه‌گیری‌ها در کنار اطلاعات IMU و انکودرهای چرخ برای تخمین وضعیت ربات‌ها مورد استفاده قرار خواهند گرفت.

بنابراین معماری نهایی مکان‌یابی بر پایه‌ی اندازه‌گیری‌های نسبی و تخمین وضعیت توزیع‌شده شکل می‌گیرد، نه RTLS متداول مبتنی بر Anchorهای ثابت.

### کنترل

معماری کنترل پروژه به‌صورت چندلایه طراحی شده است.

لایه‌های پایین‌تر مسئول دریافت داده‌ی حسگرها، کنترل موتورها و تضمین رفتار بلادرنگ ربات هستند. لایه‌های بالاتر وظایفی مانند ارتباط، تخمین وضعیت، هماهنگی بین ربات‌ها و کنترل آرایش را انجام می‌دهند.

هدف نهایی این است که ربات‌ها بتوانند به‌صورت مستقل حرکت کرده و در عین حفظ آرایش جمعی، از برخورد با یکدیگر جلوگیری کنند.

### وضعیت توسعه

پروژه در حال حاضر در مرحله‌ی راه‌اندازی سخت‌افزار، مشخصه‌یابی حسگرها و شناسایی سیستم قرار دارد.

برخی از بخش‌هایی که تاکنون بررسی یا تا حدی پیاده‌سازی شده‌اند عبارت‌اند از:

* پیکربندی ساخت Firmware برای چند ربات
* راه‌اندازی سخت‌افزار
* فاصله‌سنجی و مشخصه‌یابی UWB
* دریافت و ثبت داده‌های حسگرها
* ارتباط Peer-to-Peer
* پیکربندی اختصاصی هر ربات
* شناسایی اولیه‌ی سیستم موتور و حسگرها

مراحل اصلی بعدی شامل موارد زیر هستند:

* کنترل حلقه‌بسته‌ی سرعت چرخ‌ها
* کالیبراسیون حسگرها
* تخمین وضعیت و Sensor Fusion
* زمان‌بندی مقاوم ارتباط و فاصله‌سنجی UWB
* کنترل آرایش
* حرکت جمعی و اجتناب از برخورد

### ساختار مخزن

```text
anjoman-firmware/
├── include/        # تعاریف مشترک پیکربندی و Interfaceها
├── src/            # کد اصلی Firmware
├── lib/            # ماژول‌های اختصاصی پروژه
├── examples/       # کدهای آزمایشی و مشخصه‌یابی
├── scripts/        # ابزارهای تحلیل و توسعه
└── test/           # تست‌ها
```

### محیط توسعه

توسعه‌ی پروژه عمدتاً در محیط Linux و با استفاده از ابزارهای خط فرمان انجام می‌شود.

فناوری‌ها و ابزارهای اصلی عبارت‌اند از:

* C++
* Arduino Framework
* ESP32-S3
* PlatformIO
* FreeRTOS
* UWB / DW1000
* IMU و Wheel Encoder
* ESP-NOW
* Git

### رویکرد پروژه

انجمن به‌عنوان یک سیستم رباتیکی پژوهش‌محور توسعه داده می‌شود و هدف آن صرفاً اتصال و آزمایش مستقل چند حسگر نیست.

تمرکز پروژه بر موارد زیر است:

* غیرمتمرکز بودن
* رفتار بلادرنگ و قابل پیش‌بینی
* معماری نرم‌افزار متناسب با سخت‌افزار
* اندازه‌گیری کمی عملکرد سیستم
* آزمایش‌های قابل تکرار
* توسعه‌ی ماژولار
* یکپارچه‌سازی ارتباط، تخمین، کنترل و ادراک

```

این نسخه عمداً **هیچ ادعایی مبنی بر تکمیل بودن EKF، Formation Control یا TDMA نمی‌کند** و وضعیت فعلی را از roadmap جدا نگه می‌دارد؛ برای README یک پروژه‌ی در حال توسعه، این تفکیک مهم است.
```

