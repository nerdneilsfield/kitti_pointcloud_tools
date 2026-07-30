import { defineConfig, devices } from "@playwright/test";

export default defineConfig({
  testDir: "./tests",
  timeout: 30_000,
  use: {
    baseURL: process.env.KPT_WEB_URL || "http://127.0.0.1:8765",
    headless: process.env.KPT_WEB_HEADED !== "1",
    viewport: { width: 1440, height: 900 },
  },
  projects: [
    {
      name: "chromium",
      use: {
        ...devices["Desktop Chrome"],
        launchOptions: {
          executablePath: process.env.KPT_CHROMIUM_EXECUTABLE,
          args: ["--use-gl=angle", "--use-angle=swiftshader"],
        },
      },
    },
    { name: "firefox", use: { ...devices["Desktop Firefox"] } },
    { name: "webkit", use: { ...devices["Desktop Safari"] } },
  ],
});
