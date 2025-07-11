---

# 🛰️ Gyroscope Serial Monitor with PuTTY

This guide will walk you through using **PuTTY** to monitor gyroscope data via a serial connection.

---

## 🔧 Requirements

* Windows PC
* PuTTY Terminal ([Download here](https://www.putty.org/))
* A device connected via serial (e.g., a gyroscope module via USB-to-Serial)

---

## 🚀 Steps to Monitor Gyroscope Data

### 1. Download PuTTY

* Visit [https://www.putty.org/](https://www.putty.org/)
* Download and install the PuTTY terminal

### 2. Check the COM Port Number

* Open **Device Manager**
* Expand **Ports (COM & LPT)**
* Note the **COM port number** (e.g., `COM5`) associated with your device

### 3. Configure PuTTY

* Launch **PuTTY**
* In the left-hand menu, select **Session**
* Under **Connection type**, choose **Serial**
* In the **Serial line** field, enter your COM port number (e.g., `COM5`)
* Set **Speed (baud rate)** to `115200`

### 4. Connect and Monitor

* Click **Open**
* The terminal window will launch
* You should now see **gyroscope data** being displayed in real time

---

## 🧭 Troubleshooting

* Ensure the correct COM port is selected
* Verify the baud rate matches your device (typically `115200`)
* If no data appears, check your physical connections and restart PuTTY

---



