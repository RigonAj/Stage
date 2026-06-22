const CACHE_NAME = "ur3e-web-ui-v5";
const APP_SHELL = [
  "/",
  "/manifest.webmanifest",
  "/favicon.svg",
  "/static/css/app.css",
  "/static/js/api.js",
  "/static/js/main.js?v=catch-flight-time",
  "/static/js/viewer3d.js?v=catch-flight-time",
  "/static/js/jog_panel.js",
  "/static/js/target_panel.js?v=tcp-target-base-frame",
  "/static/js/rollout_panel.js?v=exec-target-ghost",
  "/static/js/calibration_panel.js",
  "/static/js/catch_panel.js?v=catch-flight-time",
  "/static/icons/ur3e-web-ui.svg",
];

self.addEventListener("install", (event) => {
  event.waitUntil(caches.open(CACHE_NAME).then((cache) => cache.addAll(APP_SHELL)));
  self.skipWaiting();
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys().then((names) =>
      Promise.all(names.filter((name) => name !== CACHE_NAME).map((name) => caches.delete(name))),
    ),
  );
  self.clients.claim();
});

self.addEventListener("fetch", (event) => {
  const request = event.request;
  const url = new URL(request.url);

  if (request.method !== "GET" || url.pathname.startsWith("/api/") || url.pathname === "/ws") {
    return;
  }

  event.respondWith(
    fetch(request)
      .then((response) => {
        const copy = response.clone();
        caches.open(CACHE_NAME).then((cache) => cache.put(request, copy));
        return response;
      })
      .catch(() => caches.match(request)),
  );
});
