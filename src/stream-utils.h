#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include "esp_camera.h"

static auto _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static auto _STREAM_BOUNDARY = "\r\n\r\n--frame\r\n";
static auto _STREAM_BOUNDARY_PART = "\r\n\r\n--frame\r\n\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static auto _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// --- streaming state machine (onAck-driven) ---

enum StreamState {
  S_HDR,    // sending boundary+headers (maybe partial)
  S_IMG,    // sending image bytes (partial allowed)
  S_TRAIL   // sending trailing "\r\n" after image (maybe partial)
};

// --- Tunables ---
static const size_t CHUNK_SIZE = 1024;
static const uint32_t MAX_BLOCK_MS = 200;      // drop frame if blocked > 200ms
static const size_t MIN_SPACE_FOR_PROGRESS = 512; // bytes of space threshold

struct StreamContext {
  AsyncClient *client;

  char headerBuf[128];
  size_t headerLen;
  size_t headerSent;

  camera_fb_t *fb;
  size_t imageSent;

  const char *trailer = "\r\n";
  size_t trailerSent;

  bool firstFrame;
  StreamState state;

  // new: track time of last progress
  uint32_t lastProgress;
};

// helper: release old frame, grab a new one
static inline void prepareHeader(StreamContext *ctx) {
  // Build per-frame header into headerBuf. For first frame omit leading \r\n.
  const char *boundaryFirst = "--frame\r\n";
  const char *boundaryNext  = "\r\n--frame\r\n";

  if (ctx->firstFrame) {
    // "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"
    ctx->headerLen = snprintf(ctx->headerBuf, sizeof(ctx->headerBuf),
                              "%sContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                              boundaryFirst, (unsigned)ctx->fb->len);
    ctx->firstFrame = false;
  } else {
    ctx->headerLen = snprintf(ctx->headerBuf, sizeof(ctx->headerBuf),
                              "%sContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                              boundaryNext, (unsigned)ctx->fb->len);
  }
  ctx->headerSent = 0;
  ctx->imageSent = 0;
  ctx->trailerSent = 0;
  ctx->state = S_HDR;
}

static inline bool refreshFrame(StreamContext *ctx) {
  if (ctx->fb) esp_camera_fb_return(ctx->fb);
  ctx->fb = esp_camera_fb_get();
  if (!ctx->fb) return false;
  prepareHeader(ctx);
  ctx->lastProgress = millis();
  return true;
}

// core function: try to write as much as possible according to current state and client->space()
void sendAvailable(StreamContext *ctx) {
  if (!ctx || !ctx->client || !ctx->client->connected()) return;

  size_t space = ctx->client->space();
  uint32_t now = millis();

  // detect prolonged stall
  if ((now - ctx->lastProgress) > MAX_BLOCK_MS && space < MIN_SPACE_FOR_PROGRESS) {
    refreshFrame(ctx);
  }

  if (space == 0) return;

  while (space > 0 && ctx->client->connected()) {
    if (ctx->state == S_HDR) {
      size_t remain = ctx->headerLen - ctx->headerSent;
      if (remain == 0) { ctx->state = S_IMG; continue; }

      size_t toWrite = (remain < space) ? remain : space;
      ctx->client->write(ctx->headerBuf + ctx->headerSent, toWrite);
      ctx->headerSent += toWrite;
      ctx->lastProgress = now;
      space -= toWrite;
      if (ctx->headerSent < ctx->headerLen) return;
      ctx->state = S_IMG;
      continue;
    }

    else if (ctx->state == S_IMG) {
      size_t remainingImg = ctx->fb->len - ctx->imageSent;
      if (remainingImg == 0) { ctx->state = S_TRAIL; continue; }

      size_t chunk = (remainingImg < CHUNK_SIZE) ? remainingImg : CHUNK_SIZE;
      if (chunk > space) chunk = space;
      if (chunk == 0) return;

      ctx->client->write(reinterpret_cast<const char*>(ctx->fb->buf + ctx->imageSent), chunk);
      ctx->imageSent += chunk;
      ctx->lastProgress = now;
      space -= chunk;

      if (ctx->imageSent < ctx->fb->len) return;
      ctx->state = S_TRAIL;
      continue;
    }

    else if (ctx->state == S_TRAIL) {
      size_t remain = 2 - ctx->trailerSent;
      if (remain == 0) {
        if (!refreshFrame(ctx)) return; // next frame unavailable
        continue;
      }
      size_t toWrite = (remain < space) ? remain : space;
      ctx->client->write(ctx->trailer + ctx->trailerSent, toWrite);
      ctx->trailerSent += toWrite;
      ctx->lastProgress = now;
      space -= toWrite;
      if (ctx->trailerSent < 2) return;
      continue;
    }
  }
}

// called when the client disconnects to cleanup
void cleanupCtx(StreamContext *ctx) {
  if (!ctx) return;
  if (ctx->fb) esp_camera_fb_return(ctx->fb);
  delete ctx;
}

// Web handler
void handleStream(AsyncWebServerRequest *request) {
  AsyncClient *client = request->client();
  if (!client) {
    request->send(500);
    return;
  }

  // send initial HTTP response header
  String hdr = "HTTP/1.1 200 OK\r\n";
  hdr += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n";
  hdr += "Cache-Control: no-cache\r\n";
  hdr += "Pragma: no-cache\r\n";
  hdr += "\r\n";
  client->write(hdr.c_str(), hdr.length());

  // Allocate context
  StreamContext *ctx = new StreamContext();
  ctx->client = client;
  ctx->firstFrame = true;

  // get first frame
  ctx->fb = esp_camera_fb_get();
  if (!ctx->fb) {
    // cannot start stream
    delete ctx;
    request->send(500);
    return;
  }
  // Prepare header buffer for first frame
  prepareHeader(ctx);

  // Kick off sending whatever space is available now
  sendAvailable(ctx);

  // onAck -> more space available, continue sending
  client->onAck([](void *arg, AsyncClient *c, size_t len, uint32_t time) {
    StreamContext *ctx = (StreamContext *)arg;
    if (!ctx) return;
    // Try to send more
    sendAvailable(ctx);
  }, ctx);

  // onDisconnect -> cleanup
  client->onDisconnect([](void *arg, AsyncClient *c) {
    StreamContext *ctx = (StreamContext *)arg;
    cleanupCtx(ctx);
  }, ctx);

  // onError -> cleanup
  client->onError([](void *arg, AsyncClient *c, int8_t error) {
    StreamContext *ctx = (StreamContext *)arg;
    cleanupCtx(ctx);
  }, ctx);
}
