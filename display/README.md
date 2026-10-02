# Display

Pilot horizon GUI for the landing-assist system. It draws the artificial horizon, actual pitch (yellow cue), suggested pitch (magenta bar), a logarithmic height tape, and battery. A Dev panel stands in for sensor values until the ESP32 is feeding live data. When Model T is not usable the instrument blanks and shows "Don't Trust Sensors".

```bash
npm install
npm run dev
```

The dev server listens on port 8080.
