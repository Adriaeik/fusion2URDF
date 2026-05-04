"""
brainrot.py
===========

Non-critical interactive trivia window for Fusion 360 scripts.

Features:
- start_brainrot() never raises
- stop_brainrot() never raises
- Windows + macOS support
- Opens a local HTML trivia quiz window
- Fetches questions from fixed OpenTDB API:
  https://opentdb.com/api.php?amount=30
- If API/network/browser/temp fails, the main Fusion task continues
- Keeps the window alive until stop_brainrot() is called, when possible
- Same public API as the old brainrot module
"""

from __future__ import annotations

import os
import sys
import time
import json
import shutil
import socket
import random
import tempfile
import threading
import subprocess
import webbrowser
import urllib.request
from dataclasses import dataclass
from html import escape, unescape
from typing import Optional


# =============================================================================
# Defaults
# =============================================================================

OPENTDB_API_URL = "https://opentdb.com/api.php?amount=30"

DEFAULT_ONLINE_URL = None

DEFAULT_TITLE = "Exporting URDF/XACRO..."
DEFAULT_MESSAGE = "Answer trivia while Fusion is working."

DEFAULT_SWITCH_INTERVAL_S = 10
DEFAULT_KEEP_ALIVE = True


# =============================================================================
# Data
# =============================================================================

@dataclass
class BrainrotHandle:
    process: Optional[subprocess.Popen] = None
    temp_dir: Optional[str] = None
    html_url: Optional[str] = None

    # Kept for backwards compatibility with older code.
    used_media_urls: Optional[list[str]] = None
    used_audio_url: Optional[str] = None

    can_close: bool = False
    keep_alive: bool = False

    stop_event: Optional[threading.Event] = None
    watcher_thread: Optional[threading.Thread] = None

    error: Optional[str] = None


# =============================================================================
# Paths and helpers
# =============================================================================

def _file_url(path: str) -> str:
    try:
        abs_path = os.path.abspath(path)
        return "file:///" + abs_path.replace("\\", "/")
    except BaseException:
        return ""


def _write_html(temp_dir: str, html: str) -> str:
    html_path = os.path.join(temp_dir, "brainrot.html")

    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html)

    return html_path


# =============================================================================
# Network
# =============================================================================

def _has_internet(timeout_s: float = 1.0) -> bool:
    """
    Fast non-critical network check.

    Never raises.
    """

    try:
        with socket.create_connection(("1.1.1.1", 53), timeout=timeout_s):
            return True
    except BaseException:
        return False


def _safe_str(value: object, fallback: str = "") -> str:
    try:
        if value is None:
            return fallback
        return str(value)
    except BaseException:
        return fallback


def _normalize_question(raw: dict) -> Optional[dict]:
    """
    Convert one OpenTDB result into internal quiz format.

    Never raises.
    """

    try:
        if not isinstance(raw, dict):
            return None

        question = unescape(_safe_str(raw.get("question"))).strip()
        correct_answer = unescape(_safe_str(raw.get("correct_answer"))).strip()

        raw_incorrect = raw.get("incorrect_answers", [])
        if not isinstance(raw_incorrect, list):
            raw_incorrect = []

        incorrect_answers = [
            unescape(_safe_str(answer)).strip()
            for answer in raw_incorrect
            if _safe_str(answer).strip()
        ]

        if not question or not correct_answer:
            return None

        answers = incorrect_answers + [correct_answer]

        # Remove empty answers and duplicates while preserving order.
        seen = set()
        unique_answers = []

        for answer in answers:
            key = answer.casefold()
            if answer and key not in seen:
                seen.add(key)
                unique_answers.append(answer)

        if len(unique_answers) < 2:
            return None

        random.shuffle(unique_answers)

        return {
            "category": unescape(_safe_str(raw.get("category"), "Trivia")),
            "difficulty": unescape(_safe_str(raw.get("difficulty"), "")),
            "type": unescape(_safe_str(raw.get("type"), "")),
            "question": question,
            "correct_answer": correct_answer,
            "answers": unique_answers,
        }

    except BaseException:
        return None


def _fallback_questions() -> list[dict]:
    """
    Local fallback if OpenTDB is unavailable.

    Never raises.
    """

    try:
        return [
            {
                "category": "Fallback",
                "difficulty": "easy",
                "type": "multiple",
                "question": "The trivia API could not be reached. What should this window do?",
                "correct_answer": "Keep running without breaking Fusion",
                "answers": [
                    "Keep running without breaking Fusion",
                    "Crash the export",
                    "Raise an exception",
                    "Delete the project",
                ],
            },
            {
                "category": "Fallback",
                "difficulty": "easy",
                "type": "boolean",
                "question": "Should a non-critical loading window ever stop the main Fusion task?",
                "correct_answer": "False",
                "answers": [
                    "True",
                    "False",
                ],
            },
        ]
    except BaseException:
        return []


def _fetch_trivia_questions() -> list[dict]:
    """
    Fetch trivia questions from the fixed OpenTDB API.

    Never raises.
    """

    try:
        if not _has_internet(timeout_s=1.0):
            return _fallback_questions()

        req = urllib.request.Request(
            OPENTDB_API_URL,
            headers={
                "User-Agent": "FusionTriviaWindow/1.0",
                "Accept": "application/json",
            },
        )

        with urllib.request.urlopen(req, timeout=5.0) as response:
            raw_bytes = response.read()

        raw_text = raw_bytes.decode("utf-8", errors="replace")
        data = json.loads(raw_text)

        if not isinstance(data, dict):
            return _fallback_questions()

        if data.get("response_code") != 0:
            return _fallback_questions()

        results = data.get("results", [])
        if not isinstance(results, list):
            return _fallback_questions()

        questions = []

        for item in results:
            normalized = _normalize_question(item)
            if normalized:
                questions.append(normalized)

        if not questions:
            return _fallback_questions()

        return questions

    except BaseException:
        return _fallback_questions()


# =============================================================================
# HTML
# =============================================================================

def _json_for_script(value: object) -> str:
    """
    Safely encode JSON for embedding inside a script tag.

    Never raises.
    """

    try:
        text = json.dumps(value, ensure_ascii=False)
        return text.replace("</", "<\\/")
    except BaseException:
        return "[]"


def _make_trivia_html(
    title: str,
    message: str,
    questions: list[dict],
) -> str:
    """
    Build local interactive trivia HTML.

    Never raises.
    """

    try:
        safe_title = escape(_safe_str(title, DEFAULT_TITLE))
        safe_message = escape(_safe_str(message, DEFAULT_MESSAGE))

        if not questions:
            questions = _fallback_questions()

        questions_json = _json_for_script(questions)

        return f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>{safe_title}</title>
<style>
    html, body {{
        margin: 0;
        width: 100%;
        min-height: 100%;
        background: radial-gradient(circle at center, #2f2f2f 0%, #111 72%);
        color: white;
        font-family: Arial, Helvetica, sans-serif;
    }}

    body {{
        display: flex;
        align-items: center;
        justify-content: center;
    }}

    .wrap {{
        width: 100%;
        max-width: 780px;
        padding: 32px;
        box-sizing: border-box;
    }}

    .card {{
        background: rgba(255, 255, 255, 0.08);
        border: 1px solid rgba(255, 255, 255, 0.14);
        border-radius: 24px;
        padding: 28px;
        box-shadow: 0 20px 80px rgba(0, 0, 0, 0.45);
    }}

    h1 {{
        margin: 0 0 8px;
        font-size: 34px;
        letter-spacing: -0.04em;
    }}

    .msg {{
        margin: 0 0 22px;
        opacity: 0.75;
        font-size: 16px;
    }}

    .meta {{
        display: flex;
        gap: 12px;
        flex-wrap: wrap;
        margin-bottom: 18px;
        font-size: 13px;
        opacity: 0.85;
    }}

    .pill {{
        padding: 6px 10px;
        border-radius: 999px;
        background: rgba(255, 255, 255, 0.12);
    }}

    .question {{
        font-size: 24px;
        line-height: 1.3;
        margin: 18px 0 22px;
    }}

    .answers {{
        display: grid;
        gap: 12px;
    }}

    button {{
        border: 0;
        border-radius: 14px;
        padding: 14px 16px;
        font-size: 16px;
        cursor: pointer;
        background: rgba(255, 255, 255, 0.92);
        color: #111;
        text-align: left;
    }}

    button:hover {{
        background: white;
    }}

    button:disabled {{
        cursor: default;
        opacity: 0.86;
    }}

    .correct {{
        background: #50d890 !important;
        color: #06130b !important;
    }}

    .wrong {{
        background: #ff6b6b !important;
        color: #1a0505 !important;
    }}

    .footer {{
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-top: 22px;
        gap: 12px;
    }}

    .next {{
        text-align: center;
        background: rgba(255, 255, 255, 0.16);
        color: white;
    }}

    .score {{
        opacity: 0.85;
    }}

    .done {{
        font-size: 24px;
        margin-top: 20px;
    }}

    .small {{
        font-size: 13px;
        opacity: 0.7;
        margin-top: 14px;
    }}
</style>
</head>
<body>
    <div class="wrap">
        <div class="card">
            <h1>{safe_title}</h1>
            <p class="msg">{safe_message}</p>
            <div id="quiz"></div>
        </div>
    </div>

<script>
(function () {{
    "use strict";

    const questions = {questions_json};

    let index = 0;
    let score = 0;
    let answered = false;

    function asText(value, fallback) {{
        try {{
            if (value === null || value === undefined) return fallback || "";
            return String(value);
        }} catch (e) {{
            return fallback || "";
        }}
    }}

    function clearElement(element) {{
        try {{
            while (element.firstChild) {{
                element.removeChild(element.firstChild);
            }}
        }} catch (e) {{}}
    }}

    function makeElement(tagName, className, text) {{
        const element = document.createElement(tagName);

        if (className) {{
            element.className = className;
        }}

        if (text !== undefined && text !== null) {{
            element.textContent = asText(text);
        }}

        return element;
    }}

    function getQuizElement() {{
        return document.getElementById("quiz");
    }}

    function getCurrentQuestion() {{
        try {{
            if (!Array.isArray(questions)) return null;
            if (index < 0 || index >= questions.length) return null;

            const q = questions[index];

            if (!q || typeof q !== "object") return null;
            if (!Array.isArray(q.answers)) return null;

            return q;
        }} catch (e) {{
            return null;
        }}
    }}

    function renderDone() {{
        const quiz = getQuizElement();
        if (!quiz) return;

        clearElement(quiz);

        const done = makeElement(
            "div",
            "done",
            "Finished. Score: " + score + " / " + questions.length
        );

        const footer = makeElement("div", "footer");

        const restartButton = makeElement("button", "next", "Restart quiz");
        restartButton.onclick = function () {{
            try {{
                index = 0;
                score = 0;
                answered = false;
                renderQuestion();
            }} catch (e) {{}}
        }};

        footer.appendChild(makeElement("div", "score", "Done"));
        footer.appendChild(restartButton);

        quiz.appendChild(done);
        quiz.appendChild(footer);
    }}

    function renderQuestion() {{
        try {{
            answered = false;

            const quiz = getQuizElement();
            if (!quiz) return;

            clearElement(quiz);

            if (!Array.isArray(questions) || questions.length === 0) {{
                quiz.appendChild(makeElement("p", "", "No questions available."));
                return;
            }}

            if (index >= questions.length) {{
                renderDone();
                return;
            }}

            const q = getCurrentQuestion();

            if (!q) {{
                index += 1;
                renderQuestion();
                return;
            }}

            const meta = makeElement("div", "meta");

            meta.appendChild(
                makeElement("span", "pill", "Question " + (index + 1) + " / " + questions.length)
            );

            if (q.category) {{
                meta.appendChild(makeElement("span", "pill", q.category));
            }}

            if (q.difficulty) {{
                meta.appendChild(makeElement("span", "pill", q.difficulty));
            }}

            const question = makeElement("div", "question", q.question || "Question missing");

            const answers = makeElement("div", "answers");

            q.answers.forEach(function (answer, answerIndex) {{
                const button = makeElement("button", "", answer);
                button.id = "answer-" + answerIndex;
                button.onclick = function () {{
                    selectAnswer(answerIndex);
                }};
                answers.appendChild(button);
            }});

            const footer = makeElement("div", "footer");
            const scoreElement = makeElement("div", "score", "Score: " + score);

            const nextButton = makeElement("button", "next", "Next");
            nextButton.onclick = function () {{
                try {{
                    index += 1;
                    renderQuestion();
                }} catch (e) {{}}
            }};

            footer.appendChild(scoreElement);
            footer.appendChild(nextButton);

            quiz.appendChild(meta);
            quiz.appendChild(question);
            quiz.appendChild(answers);
            quiz.appendChild(footer);

        }} catch (e) {{
            try {{
                const quiz = getQuizElement();
                if (quiz) {{
                    clearElement(quiz);
                    quiz.appendChild(
                        makeElement("p", "", "Trivia UI failed, but the Fusion task can continue.")
                    );
                }}
            }} catch (ignored) {{}}
        }}
    }}

    function selectAnswer(answerIndex) {{
        try {{
            if (answered) return;
            answered = true;

            const q = getCurrentQuestion();
            if (!q) return;

            const selected = q.answers[answerIndex];
            const correct = q.correct_answer;

            const buttons = document.querySelectorAll(".answers button");

            buttons.forEach(function (button, i) {{
                try {{
                    button.disabled = true;

                    if (q.answers[i] === correct) {{
                        button.classList.add("correct");
                    }}
                }} catch (e) {{}}
            }});

            if (selected === correct) {{
                score += 1;
            }} else {{
                const selectedButton = document.getElementById("answer-" + answerIndex);
                if (selectedButton) {{
                    selectedButton.classList.add("wrong");
                }}
            }}

            const scoreElement = document.querySelector(".score");
            if (scoreElement) {{
                scoreElement.textContent = "Score: " + score;
            }}

        }} catch (e) {{}}
    }}

    try {{
        renderQuestion();
    }} catch (e) {{}}
}})();
</script>
</body>
</html>
"""

    except BaseException:
        return """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Fusion working</title>
</head>
<body>
<p>Fusion is working.</p>
</body>
</html>
"""


# =============================================================================
# Browser launching
# =============================================================================

def _find_browser_windows() -> Optional[str]:
    try:
        candidates = [
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files\BraveSoftware\Brave-Browser\Application\brave.exe",
            r"C:\Program Files (x86)\BraveSoftware\Brave-Browser\Application\brave.exe",
        ]

        for path in candidates:
            try:
                if os.path.exists(path):
                    return path
            except BaseException:
                pass

        return None

    except BaseException:
        return None


def _find_browser_macos() -> Optional[str]:
    try:
        candidates = [
            "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
            "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
            "/Applications/Brave Browser.app/Contents/MacOS/Brave Browser",
            "/Applications/Chromium.app/Contents/MOS/Chromium",
            "/Applications/Chromium.app/Contents/MacOS/Chromium",
        ]

        for path in candidates:
            try:
                if os.path.exists(path):
                    return path
            except BaseException:
                pass

        return None

    except BaseException:
        return None


def _launch_chromium_app(browser_path: str, url: str, temp_dir: str) -> Optional[subprocess.Popen]:
    try:
        profile_dir = os.path.join(temp_dir, "browser_profile")

        return subprocess.Popen(
            [
                browser_path,
                "--new-window",
                "--app=" + url,
                "--user-data-dir=" + profile_dir,
                "--no-first-run",
                "--disable-extensions",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    except BaseException:
        return None


def _launch_url(url: str, temp_dir: str) -> tuple[Optional[subprocess.Popen], bool]:
    """
    Returns:
        (process, can_close)

    can_close=False means fallback browser opened, but we probably cannot close it
    or keep it alive.

    Never raises.
    """

    try:
        if sys.platform.startswith("win"):
            browser = _find_browser_windows()

            if browser:
                process = _launch_chromium_app(browser, url, temp_dir)
                if process:
                    return process, True

            try:
                webbrowser.open(url)
            except BaseException:
                pass

            return None, False

        if sys.platform == "darwin":
            browser = _find_browser_macos()

            if browser:
                process = _launch_chromium_app(browser, url, temp_dir)
                if process:
                    return process, True

            try:
                subprocess.Popen(
                    ["open", url],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except BaseException:
                pass

            return None, False

        try:
            webbrowser.open(url)
        except BaseException:
            pass

        return None, False

    except BaseException:
        return None, False


# =============================================================================
# Keep-alive watchdog
# =============================================================================

def _start_watchdog(handle: BrainrotHandle, restart_delay_s: float = 1.0) -> None:
    """
    Keeps the trivia window alive until stop_brainrot() is called.

    Only works reliably when _launch_url() returned a real process.

    Never raises.
    """

    try:
        if not handle:
            return

        if not handle.keep_alive:
            return

        if not handle.can_close:
            return

        if not handle.html_url:
            return

        if not handle.temp_dir:
            return

        stop_event = threading.Event()
        handle.stop_event = stop_event

        def _watch() -> None:
            try:
                while True:
                    try:
                        if stop_event.wait(restart_delay_s):
                            return

                        process = getattr(handle, "process", None)

                        if process is None or process.poll() is not None:
                            if stop_event.is_set():
                                return

                            new_process, can_close = _launch_url(
                                handle.html_url,
                                handle.temp_dir,
                            )

                            handle.process = new_process
                            handle.can_close = can_close

                            if not can_close:
                                return

                    except BaseException:
                        try:
                            if stop_event.wait(2.0):
                                return
                        except BaseException:
                            return

            except BaseException:
                return

        thread = threading.Thread(
            target=_watch,
            name="TriviaWindowWatchdog",
            daemon=True,
        )

        handle.watcher_thread = thread
        thread.start()

    except BaseException:
        pass


# =============================================================================
# Public API
# =============================================================================

def start_brainrot(
    online_url: Optional[str] = DEFAULT_ONLINE_URL,
    title: str = DEFAULT_TITLE,
    message: str = DEFAULT_MESSAGE,
    internet_check: bool = True,
    use_local_giffy: bool = True,
    switch_interval_s: int = DEFAULT_SWITCH_INTERVAL_S,
    keep_alive: bool = DEFAULT_KEEP_ALIVE,
) -> BrainrotHandle:
    """
    Start non-critical trivia/loading window.

    This function never raises.

    The signature is kept compatible with the old brainrot.py.
    The old GIF/audio parameters are ignored, but retained so existing calls do not break.

    Normal usage:
        brainrot = start_brainrot()

    Optional:
        brainrot = start_brainrot(keep_alive=False)
    """

    temp_dir = None

    try:
        temp_dir = tempfile.mkdtemp(prefix="fusion_trivia_")

        try:
            questions = _fetch_trivia_questions()
        except BaseException:
            questions = _fallback_questions()

        try:
            html = _make_trivia_html(
                title=title,
                message=message,
                questions=questions,
            )

            html_path = _write_html(temp_dir, html)
            url = _file_url(html_path)

            if not url:
                try:
                    if temp_dir and os.path.exists(temp_dir):
                        shutil.rmtree(temp_dir, ignore_errors=True)
                except BaseException:
                    pass

                return BrainrotHandle(
                    error="HTML setup failed: could not create file URL.",
                )

        except BaseException as e:
            try:
                if temp_dir and os.path.exists(temp_dir):
                    shutil.rmtree(temp_dir, ignore_errors=True)
            except BaseException:
                pass

            return BrainrotHandle(
                error=f"HTML setup failed: {e}",
            )

        try:
            process, can_close = _launch_url(url, temp_dir)

        except BaseException as e:
            try:
                if temp_dir and os.path.exists(temp_dir):
                    shutil.rmtree(temp_dir, ignore_errors=True)
            except BaseException:
                pass

            return BrainrotHandle(
                html_url=url,
                error=f"Browser launch failed: {e}",
            )

        handle = BrainrotHandle(
            process=process,
            temp_dir=temp_dir,
            html_url=url,
            used_media_urls=None,
            used_audio_url=None,
            can_close=can_close,
            keep_alive=keep_alive,
            error=None,
        )

        try:
            _start_watchdog(handle)
        except BaseException:
            pass

        return handle

    except BaseException as e:
        try:
            if temp_dir and os.path.exists(temp_dir):
                shutil.rmtree(temp_dir, ignore_errors=True)
        except BaseException:
            pass

        return BrainrotHandle(
            error=f"Trivia window disabled after failure: {e}",
        )


def stop_brainrot(handle: Optional[BrainrotHandle], wait_s: float = 0.8) -> None:
    """
    Stop trivia/loading window.

    This function never raises.
    """

    try:
        if not handle:
            return

        try:
            stop_event = getattr(handle, "stop_event", None)
            if stop_event:
                stop_event.set()
        except BaseException:
            pass

        try:
            watcher_thread = getattr(handle, "watcher_thread", None)
            if watcher_thread and watcher_thread.is_alive():
                watcher_thread.join(timeout=0.5)
        except BaseException:
            pass

        process = None
        temp_dir = None

        try:
            process = getattr(handle, "process", None)
        except BaseException:
            process = None

        try:
            temp_dir = getattr(handle, "temp_dir", None)
        except BaseException:
            temp_dir = None

        try:
            if process and process.poll() is None:
                process.terminate()

                deadline = time.time() + max(0.0, float(wait_s))

                while time.time() < deadline:
                    try:
                        if process.poll() is not None:
                            break
                    except BaseException:
                        break

                    try:
                        time.sleep(0.05)
                    except BaseException:
                        break

                try:
                    if process.poll() is None:
                        process.kill()
                except BaseException:
                    pass

        except BaseException:
            pass

        try:
            if temp_dir and os.path.exists(temp_dir):
                shutil.rmtree(temp_dir, ignore_errors=True)
        except BaseException:
            pass

    except BaseException:
        pass