# Display

Static pilot horizon (ADI) page for the landing-assist system.

This is a single self-contained HTML file (`index.html`). It is the page the ESP32 serves over its SoftAP so the cockpit phone can open it.

The previous React/TanStack app was removed; it was heavier than needed for a local ESP32-hosted instrument.

To preview locally, open `index.html` in a browser. The Dev panel lets you move the cues until the ESP32 is feeding live data.
