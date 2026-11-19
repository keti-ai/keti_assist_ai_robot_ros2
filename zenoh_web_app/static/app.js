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
