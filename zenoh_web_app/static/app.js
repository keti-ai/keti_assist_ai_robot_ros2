// Tab switching
document.querySelectorAll(".tab-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tab").forEach((t) => t.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
  });
});

// Home tab: show which zenoh bridge this server connects to
fetch("/api/status")
  .then((r) => r.json())
  .then((s) => {
    document.getElementById("bridge-info").textContent = `${s.zenoh_host}:${s.zenoh_port}`;
  })
  .catch(() => {
    document.getElementById("bridge-info").textContent = "조회 실패";
  });

// Video tab: scan topics, let the user toggle tiles on/off
const scanBtn = document.getElementById("scan-btn");
const scanStatus = document.getElementById("scan-status");
const topicList = document.getElementById("topic-list");
const videoGrid = document.getElementById("video-grid");
const openTiles = new Map(); // key_expr -> tile element

scanBtn.addEventListener("click", scanTopics);

function scanTopics() {
  scanStatus.textContent = "스캔 중...";
  scanBtn.disabled = true;

  fetch("/api/topics")
    .then((r) => r.json())
    .then((data) => {
      if (!data.ok) {
        scanStatus.textContent = "스캔 실패: " + data.error;
        topicList.innerHTML = "";
        return;
      }
      renderTopicList(data.topics);
      scanStatus.textContent = `${data.topics.length}개 이미지 토픽 발견`;
    })
    .catch((e) => {
      scanStatus.textContent = "스캔 실패: " + e;
    })
    .finally(() => {
      scanBtn.disabled = false;
    });
}

function renderTopicList(topics) {
  topicList.innerHTML = "";

  if (topics.length === 0) {
    const li = document.createElement("li");
    li.textContent = "이미지 토픽을 찾지 못했습니다. zenoh bridge와 ros2 bag이 실행 중인지 확인하세요.";
    topicList.appendChild(li);
    return;
  }

  topics.forEach((topic) => {
    const li = document.createElement("li");
    if (!topic.supported) li.classList.add("unsupported");

    const label = document.createElement("label");

    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.dataset.key = topic.key_expr;
    checkbox.disabled = !topic.supported;
    checkbox.checked = openTiles.has(topic.key_expr);
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) {
        openTile(topic);
      } else {
        closeTile(topic.key_expr);
      }
    });

    const resolution = topic.width && topic.height ? ` ${topic.width}x${topic.height}` : "";
    const text = document.createElement("span");
    text.innerHTML = `${topic.name} <span class="topic-type">(${topic.type}${resolution}${topic.supported ? "" : " - 미지원"})</span>`;

    label.appendChild(checkbox);
    label.appendChild(text);
    li.appendChild(label);
    topicList.appendChild(li);
  });
}

function openTile(topic) {
  if (openTiles.has(topic.key_expr)) return;

  const tile = document.createElement("div");
  tile.className = "tile";

  const header = document.createElement("div");
  header.className = "tile-header";

  const title = document.createElement("span");
  title.textContent = topic.name;

  const closeBtn = document.createElement("button");
  closeBtn.textContent = "✕";
  closeBtn.title = "닫기";
  closeBtn.addEventListener("click", () => {
    closeTile(topic.key_expr);
    const checkbox = topicList.querySelector(`input[data-key="${topic.key_expr}"]`);
    if (checkbox) checkbox.checked = false;
  });

  header.appendChild(title);
  header.appendChild(closeBtn);

  const img = document.createElement("img");
  img.src = "/api/stream?topic=" + encodeURIComponent(topic.key_expr);
  img.alt = topic.name;

  tile.appendChild(header);
  tile.appendChild(img);
  videoGrid.appendChild(tile);

  openTiles.set(topic.key_expr, tile);
}

function closeTile(keyExpr) {
  const tile = openTiles.get(keyExpr);
  if (!tile) return;
  tile.querySelector("img").src = ""; // stop the MJPEG connection
  tile.remove();
  openTiles.delete(keyExpr);
}

// initial scan on load
scanTopics();

// Message tab: scan every topic, let the user check boxes to echo their
// raw payloads live as text (useful for topics with no dedicated viewer).
const echoScanBtn = document.getElementById("echo-scan-btn");
const echoScanStatus = document.getElementById("echo-scan-status");
const echoTopicList = document.getElementById("echo-topic-list");
const echoGrid = document.getElementById("echo-grid");
const openEchoTiles = new Map(); // key_expr -> { tile, textarea, ws }

echoScanBtn.addEventListener("click", scanAllTopics);

function scanAllTopics() {
  echoScanStatus.textContent = "스캔 중...";
  echoScanBtn.disabled = true;

  fetch("/api/all_topics")
    .then((r) => r.json())
    .then((data) => {
      if (!data.ok) {
        echoScanStatus.textContent = "스캔 실패: " + data.error;
        echoTopicList.innerHTML = "";
        return;
      }
      renderEchoTopicList(data.topics);
      echoScanStatus.textContent = `${data.topics.length}개 토픽 발견`;
    })
    .catch((e) => {
      echoScanStatus.textContent = "스캔 실패: " + e;
    })
    .finally(() => {
      echoScanBtn.disabled = false;
    });
}

function renderEchoTopicList(topics) {
  echoTopicList.innerHTML = "";

  if (topics.length === 0) {
    const li = document.createElement("li");
    li.textContent = "토픽을 찾지 못했습니다. zenoh bridge가 실행 중인지 확인하세요.";
    echoTopicList.appendChild(li);
    return;
  }

  topics.forEach((topic) => {
    const li = document.createElement("li");

    const label = document.createElement("label");

    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.dataset.key = topic.key_expr;
    checkbox.checked = openEchoTiles.has(topic.key_expr);
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) {
        openEchoTile(topic);
      } else {
        closeEchoTile(topic.key_expr);
      }
    });

    const text = document.createElement("span");
    text.innerHTML = `${topic.name} <span class="topic-type">(${topic.type})</span>`;

    label.appendChild(checkbox);
    label.appendChild(text);
    li.appendChild(label);
    echoTopicList.appendChild(li);
  });
}

function openEchoTile(topic) {
  if (openEchoTiles.has(topic.key_expr)) return;

  const tile = document.createElement("div");
  tile.className = "tile echo-tile";

  const header = document.createElement("div");
  header.className = "tile-header";

  const title = document.createElement("span");
  title.textContent = topic.name;

  const closeBtn = document.createElement("button");
  closeBtn.textContent = "✕";
  closeBtn.title = "닫기";
  closeBtn.addEventListener("click", () => {
    closeEchoTile(topic.key_expr);
    const checkbox = echoTopicList.querySelector(`input[data-key="${topic.key_expr}"]`);
    if (checkbox) checkbox.checked = false;
  });

  header.appendChild(title);
  header.appendChild(closeBtn);

  const textarea = document.createElement("textarea");
  textarea.className = "echo-box";
  textarea.readOnly = true;
  textarea.placeholder = "메시지를 기다리는 중...";

  tile.appendChild(header);
  tile.appendChild(textarea);
  echoGrid.appendChild(tile);

  const proto = location.protocol === "https:" ? "wss" : "ws";
  const params = new URLSearchParams({ topic: topic.key_expr, type: topic.type });
  const ws = new WebSocket(`${proto}://${location.host}/ws/topic_echo?${params}`);

  ws.addEventListener("message", (ev) => {
    const msg = JSON.parse(ev.data);
    const time = new Date(msg.timestamp * 1000).toLocaleTimeString("ko-KR", { hour12: false });
    const header = `[${time}] (${msg.size}B)`;
    const block = msg.text.includes("\n") ? `${header}\n${msg.text}` : `${header} ${msg.text}`;
    textarea.value += block + "\n\n";

    const lines = textarea.value.split("\n");
    if (lines.length > 400) {
      textarea.value = lines.slice(lines.length - 400).join("\n");
    }
    textarea.scrollTop = textarea.scrollHeight;
  });

  ws.addEventListener("error", () => {
    textarea.value += "[연결 오류]\n";
  });

  openEchoTiles.set(topic.key_expr, { tile, textarea, ws });
}

function closeEchoTile(keyExpr) {
  const entry = openEchoTiles.get(keyExpr);
  if (!entry) return;
  entry.ws.close();
  entry.tile.remove();
  openEchoTiles.delete(keyExpr);
}
