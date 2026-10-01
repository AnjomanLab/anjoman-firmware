# Anjoman Firmware — انجمن

[فارسی](#فارسی) | [English](#english)

---
<div dir="rtl" style="text-align: justify;">
<a name="فارسی"></a>

## انجمن

**انجمن** یک بستر تحقیقاتی برای توسعه‌ی ناوگان‌هایی از ربات‌های متحرک خودران با مکانیزم حرکتی دیفرانسیل‌درایو است. هدف پروژه، دستیابی به **مکان‌یابی نسبی غیرمتمرکز** و **کنترل آرایشی دقیق**، بدون اتکا به Anchor ثابت UWB، GPS، یا کامپیوتر مرکزی است.

نام پروژه از مفهوم **انجمن** به معنای گردهمایی و همکاری مجموعه‌ای از عامل‌های مستقل الهام گرفته شده است.

---

### ماهیت پروژه

پروژه یک swarm رباتیکی غیرمتمرکز است که هر ربات به‌عنوان یک عامل مستقل عمل می‌کند و با همسایگان خود از طریق ESP-NOW و رنجینگ UWB تبادل اطلاعات می‌کند. این معماری به‌گونه‌ای طراحی شده که حتی در صورت از دست رفتن یکی از گره‌ها، بقیه‌ی ناوگان بتوانند به کار خود ادامه دهند.

**غیرمتمرکز بودن در این پروژه دو معنی مستقل دارد:**

۱. **استقلال محاسباتی:** هر ربات مستقلاً موقعیت خودش را تخمین می‌زند و مسیر خودش را کنترل می‌کند. هیچ نود مرکزی محاسبات را انجام نمی‌دهد.

۲. **استقلال از زیرساخت خارجی:** هیچ Anchor، GPS، یا فرستنده‌ی کمکی خارج از swarm وجود ندارد.

---

### اهداف سیستم

* هماهنگی غیرمتمرکز چند ربات
* مکان‌یابی نسبی با استفاده از فاصله‌سنجی UWB بین ربات‌ها
* ترکیب اطلاعات UWB، IMU و انکودرهای چرخ
* کنترل خودران ربات‌های Differential Drive
* کنترل آرایش با دقت زیر‌سانتی‌متر
* زمان‌بندی قطعی و بلادرنگ
* ارتباط Peer-to-Peer بین ربات‌ها
* پشتیبانی از سخت‌افزار ناهمگن ربات‌ها
* اجتناب از برخورد و حرکت جمعی

---

### پلتفرم ربات‌ها

ناوگان فعلی شامل **سه ربات فعال** است که روی راس‌های یک مثلث متساوی‌الاضلاع به ضلع ۲ متر قرار می‌گیرند. ربات چهارم (R1) به‌عنوان **Gateway ESP-NOW** در مرکز swarm عمل می‌کند و داده‌ها را به‌صورت آنلاین روی USB Serial چاپ می‌کند.

سخت‌افزار هر ربات:

| ماژول | مدل | اتصال |
|-------|-----|--------|
| میکروکنترلر | ESP32-S3 دو‌هسته‌ای | — |
| UWB | DWM1000 (Qorvo) | SPI اختصاصی |
| IMU | BMI160 (Bosch) | I2C (TCA9548A ch 2) |
| انکودر چپ | AS5600 | I2C (TCA9548A ch 0) |
| انکودر راست | AS5600 | I2C (TCA9548A ch 1) |
| مانیتور توان | INA226 | I2C (TCA9548A ch 3) |
| درایور موتور | DRV8833 | GPIO + LEDC |

آنتن‌های UWB به‌صورت **عمودی** (رو به آسمان) قرار گرفته‌اند تا الگوی تابش `Azimuth Theta` داشته باشند.

**ناهمگنی عمدی:** ربات‌ها از نظر ابعاد چرخ، عرض مسیر مؤثر، پارامترهای موتور، بایاس‌های IMU، و حتی مدل تراشه‌ی ESP32-S3 متفاوتند. این تفاوت‌ها در `RobotConfig.h` برای هر ربات به‌صورت جداگانه لحاظ شده است.

---

### معماری نرم‌افزار

نرم‌افزار بر پایه‌ی **FreeRTOS** و پردازنده‌ی دو‌هسته‌ای ESP32-S3 بنا شده است:

**Core 1 (۱۰۰ Hz) — حلقه‌ی بلادرنگ سخت:**
* خواندن انکودرها، IMU، INA226
* اجرای ESKF محلی
* محاسبه‌ی اودومتری
* کنترل PI + Feedforward موتورها

**Core 0 — لایه‌های بالاتر:**
* ۱۰ Hz: کنترلر آرایشی و مرجع مسیر
* ۴.۱۷ Hz: زمان‌بندی TDMA
* On-demand: ESP-NOW beacon، UWB poll/listen

ارتباط بین دو هسته از طریق ساختار `g_shared` با قفل سخت‌افزاری `portMUX` انجام می‌شود. تمام تراکنش‌های I2C از یک mutex سراسری در ماژول `AnjomanI2C` عبور می‌کنند.

---

### مکان‌یابی و تخمین حالت

#### اودومتری محلی — ESKF چهار‌حالته

هر ربات یک **Error-State Kalman Filter** چهار‌حالته دارد:

$$x_{\text{nom}} = [p_x, p_y, \theta, b_g]^T, \quad \delta x = [\delta p_x, \delta p_y, \delta\theta, \delta b_g]^T$$

**پیش‌بینی:** از مدل غیرخطی سینماتیک و قرائت ژیروسکوپ.
**به‌روزرسانی:** با افزایش زاویه‌ی انکودر دیفرانسیل.
**گیتینگ:** با فاصله‌ی Mahalanobis ($\chi^2$ در سطح ۹۹٪) و آستانه‌ی divergence نرخ زاویه‌ای، برای مقاومت در برابر slip چرخ.
**پایداری عددی:** Joseph form برای به‌روزرسانی کوواریانس.

دقت تجربی: خطای شعاعی < ۰.۵ سانتی‌متر در مانور ۱۸۰ ثانیه‌ای؛ خطای موقعیت نهایی < ۱ سانتی‌متر در مانور ۳۰ ثانیه‌ای.

#### UWB — مدل بایاس static

از روش **Two-Way Ranging متقارن (SS-TWR)** استفاده می‌شود. زمان پرواز:

$$\text{ToF} = \frac{t_{\text{round}} - t_{\text{reply}}}{2}$$

فاصله‌ی خام با یک **مدل بایاس static per-directed-link** تصحیح می‌شود:

$$d_{\text{corrected}} = d_{\text{raw}} - b_{\text{static}}(i,j)$$

که $b_{\text{static}}$ از کالیبراسیون استاتیک ۱۲-یاله (مربع ۲ متری، ۱۰ دقیقه) استخراج شده است. RMSE مدل: ۱۳.۸۳ سانتی‌متر.

**مدل حرارتی حذف شده است** چون:
* رجیستر SAR روی ماژول‌های فعلی صفر برمی‌گرداند.
* proxy دمای ESP32 در تست‌ها بایاس را بدتر می‌کند.
* در بازه‌ی ۲۰-۳۵°C، اثر حرارتی زیر ۵ سانتی‌متر است — زیر نویز اندازه‌گیری.

---

### کنترل

#### مرجع مسیر — Quintic Polynomial

مسیر مطلوب هر ربات با یک **چندجمله‌ای درجه پنج** تولید می‌شود:

$$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau \in [0,1]$$

این یک پروفایل **minimum-jerk** است: سرعت و شتاب در دو انتها صفر است، بدون پرش — که برای ربات چرخ‌دار و جلوگیری از slip حیاتی است.

موقعیت مطلوب هر ربات:
$$p_i^*(\tau) = R(\phi(\tau)) \cdot [L(\tau) \cdot r_i]$$

#### کنترلر آرایشی — Virtual Look-Ahead

مسئله‌ی غیر‌هولونومیک با تعریف یک **نقطه‌ی مجازی** به فاصله‌ی $L_p$ جلوتر از محور چرخ‌ها حل می‌شود:

$$v_{\text{cmd}} = \cos\theta \cdot u_x + \sin\theta \cdot u_y$$
$$\omega_{\text{cmd}} = \frac{-\sin\theta \cdot u_x + \cos\theta \cdot u_y}{L_p}$$

با $u_x = v_x^* + K_p(x^* - x)$ و به‌طور مشابه برای $y$. پارامترها: $L_p = 50$ mm، $K_p = 2.0$ 1/s.

#### کنترلر موتور — Feedforward + PI

از مدل استاتیک موتور (استخراج از SysID):
$$\text{RPM} = K_m \cdot (\text{PWM} - \text{DB})$$

ورودی کنترلی:
$$u = u_{\text{ff}} + K_p e + K_i \int e$$

که $u_{\text{ff}}$ ددبند و ولتاژ باتری را جبران می‌کند. D حذف شده چون نویز انکودر را تقویت می‌کند و FF نقش میرایی را ایفا می‌کند.

---

### زمان‌بندی TDMA غیرمتمرکز

فریم ۲۴۰ میلی‌ثانیه، ۱۶ اسلات ۱۵ میلی‌ثانیه‌ای. هر ربات در اسلات اختصاصی خودش یک **Sync Beacon** می‌فرستد و در اسلات‌های دیگر به poll/listen می‌پردازد.

**تخصیص اسلات (۳ رباته):**
* R2: اسلات ۰ (beacon)، ۱ (→R3)، ۲ (→R4)، ۳ (margin)
* R3: اسلات ۴ (beacon)، ۵ (→R2)، ۶ (→R4)، ۷ (margin)
* R4: اسلات ۸ (beacon)، ۹ (→R2)، ۱۰ (→R3)، ۱۱ (margin)
* اسلات ۱۲-۱۵: پنجره‌های listen و margin

**هیچ master مرکزی وجود ندارد.** هر ربات `frameStartUs` خودش را با دریافت beacon از هر peer تنظیم می‌کند. اسلات‌های margin برای جذب drift و jitter.

**Watchdog محلی:** اگر > ۵۰۰ ms هیچ beacon نرسد، `syncLost = true` می‌شود.

**Consensus برای شروع مانور:** هر ربات منتظر می‌ماند تا از همه‌ی peerها beacon دریافت کند، سپس یک فریم شروع پیشنهاد می‌دهد. همه به max این پیشنهادها همگرا می‌شوند.

---

### Gateway ESP-NOW

ربات چهارم (R1) به‌دلیل نقص سخت‌افزاری UWB از swarm حذف شده و به **Gateway** تبدیل شده است. firmware آن فقط ESP-NOW را فعال می‌کند و هر beacon دریافتی را به‌صورت یک خط CSV روی USB Serial چاپ می‌کند.

این معماری مزایایی دارد:
* جمع‌آوری آنلاین داده‌ی هر سه ربات در یک فایل
* همگام‌سازی خودکار timestamp (چون همه از یک clock می‌آیند)
* عدم نیاز به WiFi یا download از هر ربات جداگانه

فرمول CSV شامل: زمان دریافت، senderId، شماره فریم، و دو بلوک UWB با peerهای هر ربات است.

---

### وضعیت توسعه

پروژه از نظر کارشناسی **بیش از ۹۰٪ کامل** است.

**انجام‌شده:**
* راه‌اندازی سخت‌افزار چهار ربات ناهمگن
* کالیبراسیون موتور (ددبند، بهره، ثابت زمانی) per-robot
* کالیبراسیون استاتیک UWB روی ۱۲ لینک
* ESKF محلی با دقت زیر‌سانتی‌متر
* کنترلر آرایشی مبتنی بر quintic + virtual look-ahead
* TDMA غیرمتمرکز با watchdog
* Gateway ESP-NOW
* مانورهای تجربی موفق با خطای نهایی زیر ۱ سانتی‌متر

**در حال پیشرفت / باقی‌مانده:**
* EKF تعاونی برای جذب UWB در حلقه‌ی کنترل
* Consensus Time Synchronization کامل
* کالیبراسیون antenna delay per-module
* تحلیل مشاهده‌پذیری بایاس‌ها

---

### ساختار مخزن

```text
anjoman-firmware/
├── lib/                    # ماژول‌های اختصاصی پروژه
│   ├── AnjomanCommon/      # ساختارهای مشترک داده
│   ├── AnjomanConfig/      # پیکربندی per-robot
│   ├── AnjomanFormation/   # مرجع مسیر و کنترلر آرایشی
│   ├── AnjomanI2C/         # mutex سراسری I2C
│   ├── AnjomanLogging/     # لاگ‌گیر (غیرفعال در نسخه‌ی فعلی)
│   ├── AnjomanStateEstimation/  # ESKF و UWBPreprocessor
│   ├── AnjomanTDMA/        # زمان‌بندی و watchdog
│   ├── BMI160_Custom/      # درایور IMU
│   ├── Comms/              # ESP-NOW
│   ├── DW1000_Custom/      # درایور UWB
│   ├── INA226_Custom/      # مانیتور توان
│   ├── MagneticEncoder/    # درایور انکودر
│   └── MotorController/    # کنترلر موتور
└── src/
    ├── main.cpp            # firmware ربات (R2, R3, R4)
    └── gateway.cpp         # firmware gateway (R1)
```

---

### محیط توسعه

* C++ (Arduino framework)
* ESP32-S3 (espressif32@7.1.3، Arduino Core 2.x)
* PlatformIO
* FreeRTOS
* UWB / DW1000
* ESP-NOW
* BMI160 IMU + AS5600 Encoders
* Git

**کامپایل و فلش ربات‌ها:**
```bash
pio run -e r2 -t upload
pio run -e r3 -t upload
pio run -e r4 -t upload
```

**کامپایل و فلش gateway (R1):**
```bash
pio run -e gateway -t upload
pio device monitor -e gateway -b 460800 > gateway_log.csv
```

---

### رویکرد پروژه

انجمن به‌عنوان یک سیستم رباتیکی پژوهش‌محور توسعه می‌یابد، نه مجموعه‌ای از آزمایش‌های مستقل. تمرکز بر موارد زیر است:

* **غیرمتمرکز بودن** در پردازش و تصمیم‌گیری
* **سادگی قبل از پیچیدگی** — هر الگوریتم باید توجیه فنی داشته باشد
* **صداقت در مورد محدودیت‌ها** — هر محدودیت سخت‌افزاری یا الگوریتمی صریحاً مستند می‌شود
* **اندازه‌گیری کمی عملکرد** — همه‌ی اعداد از داده‌ی تجربی می‌آیند
* **آزمایش‌های قابل تکرار** — تنظیمات و شرایط هر تست ثبت می‌شود
* **توسعه‌ی ماژولار** — هر بخش از کد مستقل، قابل تست، و قابل جایگزینی است



</div>

---

<a name="english"></a>

## Anjoman Firmware — English

**Anjoman** is a research platform for developing swarms of autonomous differential-drive mobile robots. The project aims at **decentralized relative localization** and **precise formation control**, without relying on fixed UWB anchors, GPS, or any central computer.

The name **Anjoman** (meaning "gathering" in Persian) reflects the concept of a group of independent agents coordinating through cooperation rather than centralized command.

---

### Project Nature

Anjoman is a **decentralized robotic swarm** in which every robot acts as an autonomous peer, exchanging information with neighbors via ESP-NOW and UWB ranging. The architecture is designed so that the loss of any single node does not stop the rest of the fleet.

**Decentralization in this project has two independent meanings:**

1. **Computational independence:** Every robot estimates its own state and controls its own trajectory. No central node performs any computation.

2. **Infrastructure independence:** No anchors, no GPS, no external helper transmitter.

---

### System Goals

* Decentralized multi-robot coordination
* Relative localization using inter-robot UWB ranging
* Sensor fusion of UWB, IMU, and wheel odometry
* Autonomous differential-drive motion control
* Formation control with sub-centimeter accuracy
* Deterministic and real-time execution
* Peer-to-peer communication
* Support for heterogeneous robot hardware
* Collision avoidance and cooperative motion

---

### Robot Platform

The current fleet consists of **three active robots** placed on the vertices of an equilateral triangle with 2 m sides. A fourth robot (R1) acts as an **ESP-NOW Gateway** at the swarm center and streams all received beacons to USB Serial as CSV.

Robot hardware per unit:

| Module | Model | Interface |
|--------|-------|-----------|
| MCU | ESP32-S3 dual-core | — |
| UWB | DWM1000 (Qorvo) | Dedicated SPI |
| IMU | BMI160 (Bosch) | I2C (TCA9548A ch 2) |
| Left encoder | AS5600 | I2C (TCA9548A ch 0) |
| Right encoder | AS5600 | I2C (TCA9548A ch 1) |
| Power monitor | INA226 | I2C (TCA9548A ch 3) |
| Motor driver | DRV8833 | GPIO + LEDC |

UWB antennas are mounted **vertically** (pointing up) to obtain the `Azimuth Theta` radiation pattern.

**Deliberate heterogeneity:** Robots differ in wheel dimensions, effective track width, motor parameters, IMU biases, and even ESP32-S3 chip model. These differences are explicitly handled in `RobotConfig.h` per robot.

---

### Software Architecture

The firmware is built on **FreeRTOS** and the dual-core ESP32-S3:

**Core 1 (100 Hz) — hard real-time loop:**
* Encoders, IMU, INA226
* Local ESKF
* Odometry integration
* PI + Feedforward motor control

**Core 0 — higher layers:**
* 10 Hz: formation controller and reference trajectory
* 4.17 Hz: TDMA scheduling
* On-demand: ESP-NOW beacons, UWB poll/listen

Inter-core communication is through a `g_shared` structure protected by a `portMUX` critical section. All I2C transactions pass through a global mutex in the `AnjomanI2C` module.

---

### Localization and State Estimation

#### Local Odometry — 4-state ESKF

Each robot runs an **Error-State Kalman Filter** with 4 states:

$$x_{\text{nom}} = [p_x, p_y, \theta, b_g]^T, \quad \delta x = [\delta p_x, \delta p_y, \delta\theta, \delta b_g]^T$$

**Predict:** nonlinear kinematic model + gyro reading.
**Update:** differential encoder angular increment.
**Gating:** Mahalanobis distance ($\chi^2$ at 99%) and rate-divergence threshold for wheel-slip protection.
**Numerical stability:** Joseph form for covariance update.

Experimental accuracy: radial error < 0.5 cm over a 180 s maneuver; final position error < 1 cm over a 30 s maneuver.

#### UWB — Static Bias Model

Symmetric Two-Way Ranging (SS-TWR). Time of flight:

$$\text{ToF} = \frac{t_{\text{round}} - t_{\text{reply}}}{2}$$

Raw distance is corrected by a **per-directed-link static bias** model:

$$d_{\text{corrected}} = d_{\text{raw}} - b_{\text{static}}(i,j)$$

$b_{\text{static}}$ is derived from a 12-link static calibration (2 m square, 10 min). Model RMSE: 13.83 cm.

**The thermal model has been removed** because:
* The SAR register on these modules returns 0.
* ESP32 proxy temperature worsened bias in tests.
* In the 20-35 °C range, thermal effect is below 5 cm — under the measurement noise floor.

---

### Control

#### Reference Trajectory — Quintic Polynomial

$$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau \in [0,1]$$

A **minimum-jerk** profile with zero velocity and acceleration at both ends — critical for a wheeled robot to avoid slip.

Target position:

$$p_i^*(\tau) = R(\phi(\tau)) \cdot [L(\tau) \cdot r_i]$$

#### Formation Controller — Virtual Look-Ahead

The nonholonomic constraint is handled via a **virtual point** at distance $L_p$ ahead of the axle:

$$v_{\text{cmd}} = \cos\theta \cdot u_x + \sin\theta \cdot u_y$$
$$\omega_{\text{cmd}} = \frac{-\sin\theta \cdot u_x + \cos\theta \cdot u_y}{L_p}$$

With $u_x = v_x^* + K_p(x^* - x)$ and similarly for $y$. Parameters: $L_p = 50$ mm, $K_p = 2.0$ 1/s.

#### Motor Controller — Feedforward + PI

Static motor model from SysID:

$$\text{RPM} = K_m \cdot (\text{PWM} - \text{DB})$$

Control input:

$$u = u_{\text{ff}} + K_p e + K_i \int e$$

$u_{\text{ff}}$ compensates deadband and battery voltage. D is omitted because it amplifies encoder noise, and feedforward provides the required damping.

---

### Decentralized TDMA

240 ms frame, 16 slots of 15 ms. Each robot broadcasts a **Sync Beacon** in its own slot and polls/listens in others.

**Slot allocation (3-robot fleet):**
* R2: slots 0 (beacon), 1 (→R3), 2 (→R4), 3 (margin)
* R3: slots 4 (beacon), 5 (→R2), 6 (→R4), 7 (margin)
* R4: slots 8 (beacon), 9 (→R2), 10 (→R3), 11 (margin)
* Slots 12-15: listen windows and margins

**No central master.** Each robot adjusts its local frame start upon receiving any peer beacon. Margin slots absorb drift and jitter.

**Local watchdog:** if no beacon is received for 500 ms, `syncLost` is raised.

**Maneuver start consensus:** each robot waits to receive beacons from all peers, then proposes a start frame. All converge to the maximum proposal.

---

### ESP-NOW Gateway

Robot R1 has been removed from the swarm due to a UWB hardware defect and repurposed as a **Gateway**. Its firmware only enables ESP-NOW and prints each received beacon as a CSV row over USB Serial.

Benefits:
* Online data collection from all three robots in one file
* Automatic timestamp synchronization (single clock source)
* No need for WiFi or per-robot downloads

CSV format: receive time, sender ID, frame ID, and two UWB peer blocks.

---

### Development Status

The project is over **90% complete for undergraduate scope**.

**Completed:**
* Hardware bring-up of four heterogeneous robots
* Per-robot motor calibration (deadband, gain, time constant)
* Static UWB calibration over 12 directed links
* Local ESKF with sub-centimeter accuracy
* Formation controller based on quintic + virtual look-ahead
* Decentralized TDMA with watchdog
* ESP-NOW Gateway
* Successful experimental maneuvers with final position error < 1 cm

**In progress / remaining:**
* Cooperative EKF to fuse UWB into the control loop
* Full Consensus Time Synchronization
* Per-module antenna delay calibration
* Observability analysis of bias states

---

### Repository Structure

```text
anjoman-firmware/
├── lib/                    # Project-specific modules
│   ├── AnjomanCommon/      # Shared data structures
│   ├── AnjomanConfig/      # Per-robot configuration
│   ├── AnjomanFormation/   # Reference trajectory and formation controller
│   ├── AnjomanI2C/         # Global I2C mutex
│   ├── AnjomanLogging/     # Logger (disabled in current version)
│   ├── AnjomanStateEstimation/  # ESKF and UWBPreprocessor
│   ├── AnjomanTDMA/        # Scheduling and watchdog
│   ├── BMI160_Custom/      # IMU driver
│   ├── Comms/              # ESP-NOW
│   ├── DW1000_Custom/      # UWB driver
│   ├── INA226_Custom/      # Power monitor
│   ├── MagneticEncoder/    # Encoder driver
│   └── MotorController/    # Motor controller
└── src/
    ├── main.cpp            # Robot firmware (R2, R3, R4)
    └── gateway.cpp         # Gateway firmware (R1)
```

---

### Development Environment

* C++ (Arduino framework)
* ESP32-S3 (espressif32@7.1.3, Arduino Core 2.x)
* PlatformIO
* FreeRTOS
* UWB / DW1000
* ESP-NOW
* BMI160 IMU + AS5600 Encoders
* Git

**Build and flash robots:**
```bash
pio run -e r2 -t upload
pio run -e r3 -t upload
pio run -e r4 -t upload
```

**Build and flash the gateway (R1):**
```bash
pio run -e gateway -t upload
pio device monitor -e gateway -b 460800 > gateway_log.csv
```

---

### Project Philosophy

Anjoman is developed as a research-oriented robotics system rather than a collection of independent sensor demonstrations. The emphasis is on:

* **Decentralization** in computation and decision-making
* **Simplicity before complexity** — every algorithm must have a technical justification
* **Honesty about limitations** — every hardware or algorithmic limitation is explicitly documented
* **Quantitative performance measurement** — all numbers come from experimental data
* **Reproducible experiments** — test conditions and configurations are recorded
* **Modular development** — each part of the code is independent, testable, and replaceable

