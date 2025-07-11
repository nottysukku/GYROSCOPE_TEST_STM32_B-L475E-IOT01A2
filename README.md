---

# 🛰️ Gyroscope Serial Monitor with PuTTY

This guide will walk you through using **PuTTY** to monitor gyroscope data via a serial connection.

## 🔧 Requirements

* Windows PC
* PuTTY Terminal ([Download here](https://www.putty.org/))
* Device connected via serial (e.g. Gyroscope module via USB-to-Serial)

---

## 🚀 Steps to Monitor Gyro Data

### 1. Download PuTTY

* Go to [https://www.putty.org/](https://www.putty.org/)
* Download and install the PuTTY terminal

### 2. Check COM Port Number

* Open **Device Manager**
* Expand **Ports (COM & LPT)**
* Note the **COM port number** (e.g., `COM5`) of the connected device

### 3. Configure PuTTY

* Launch **PuTTY**
* In the left menu, select **Session**
* Under **Connection type**, choose **Serial**
* In the **Serial line** field, enter your COM port number (e.g., `COM5`)
* Set **Speed (baud rate)** to `115200`

### 4. Connect

* Click **Open**
* The terminal will open
* You should start seeing **gyroscope data** displayed

---

## 🧭 Troubleshooting

* Make sure the correct COM port is selected
* Ensure the baud rate matches your device (commonly 115200)
* If nothing shows up, check your connections and restart PuTTY

---

## 📷 Screenshot Example (Optional)

*You can add a screenshot of the Device Manager and PuTTY config here for better clarity.*

---
