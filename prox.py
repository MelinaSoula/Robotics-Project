if heart_rate is not None and spo2 is not None:
    now = time()

    # 🟡 1. Detect environmental abnormalities
    env_alerts = []
    if co2 is not None and co2 > CO2_THRESHOLD:
        env_alerts.append(f"⚠️ CO2 High: {co2} ppm")
    if temp is not None:
        if temp > TEMP_HIGH_THRESHOLD:
            env_alerts.append(f"🔥 High Temp: {temp}°C")
        elif temp < TEMP_LOW_THRESHOLD:
            env_alerts.append(f"❄️ Low Temp: {temp}°C")
    if hum is not None and hum > HUMIDITY_THRESHOLD:
        env_alerts.append(f"💧 High Humidity: {hum}%")
    if tvoc is not None and tvoc > TVOC_THRESHOLD:
        env_alerts.append(f"⚠️ High TVOC: {tvoc} ppb")

    # 🔴 2. Detect heart condition alerts
    heart_alerts = []
    if heart_rate > 100:
        heart_alerts.append(f"⚠️ High BPM: {heart_rate} bpm (Tachycardia)")
    elif heart_rate < 60:
        heart_alerts.append(f"⚠️ Low BPM: {heart_rate} bpm (Bradycardia)")
    if spo2 < 95:
        heart_alerts.append(f"⚠️ Low SpO₂: {spo2}%")

    # 🔵 3. Decide who to alert and beep appropriately
    if heart_alerts and not env_alerts:
        if abnormal_start_times["patient_condition"] is None:
            abnormal_start_times["patient_condition"] = now
        elif (now - abnormal_start_times["patient_condition"] >= ALERT_STABLE_DURATION and
              now - last_alert_times["patient_condition"] >= ALERT_COOLDOWN):
            BLYNK.log_event("patient_condition", "\n".join(heart_alerts))
            buzzer_beep('doctor')
            last_alert_times["patient_condition"] = now

    elif env_alerts and not heart_alerts:
        if abnormal_start_times["room_conditions"] is None:
            abnormal_start_times["room_conditions"] = now
        elif (now - abnormal_start_times["room_conditions"] >= ALERT_STABLE_DURATION and
              now - last_alert_times["room_conditions"] >= ALERT_COOLDOWN):
            BLYNK.log_event("room_conditions", "\n".join(env_alerts))
            buzzer_beep('nurse_env')
            last_alert_times["room_conditions"] = now

    elif env_alerts and heart_alerts:
        if abnormal_start_times["patient_discomfort"] is None:
            abnormal_start_times["patient_discomfort"] = now
        elif (now - abnormal_start_times["patient_discomfort"] >= ALERT_STABLE_DURATION and
              now - last_alert_times["patient_discomfort"] >= ALERT_COOLDOWN):
            all_alerts = heart_alerts + env_alerts
            BLYNK.log_event("patient_discomfort", "\n".join(all_alerts))
            buzzer_beep('nurse_both')
            last_alert_times["patient_discomfort"] = now

    # 🔘 Reset timers when conditions are safe again
    if not heart_alerts:
        abnormal_start_times["patient_condition"] = None
    if not env_alerts:
        abnormal_start_times["room_conditions"] = None
    if not heart_alerts or not env_alerts:
        abnormal_start_times["patient_discomfort"] = None
