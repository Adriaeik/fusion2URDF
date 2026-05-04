"""
Logger — Standalone logging for Fusion 360 scripts.

Fusion 360 has no terminal. This module collects log lines in memory
and saves them as Markdown or plain text. Supports sections, indentation,
and timestamp control.

Usage:
    from .core.logger import Logger
    log = Logger()
    log("Starting export")
    log.section("JOINTS")
    log("  Found 5 joints")
    log.save_markdown("/path/to/export_log.md", title="Export Log")

Author: Adrian Valaker Eikeland
"""

from datetime import datetime
from typing import List, Optional


class Logger:
    """In-memory logger that saves to file."""
    
    def __init__(self, timestamps: bool = True, quiet: bool = False):
        self._lines: List[str] = []
        self._timestamps = timestamps
        self._quiet = quiet
    
    def __call__(self, msg: str):
        """Log a message. Main interface — use like a function."""
        if self._timestamps:
            ts = datetime.now().strftime("%H:%M:%S")
            line = f"[{ts}] {msg}"
        else:
            line = msg
        self._lines.append(line)
        if not self._quiet:
            print(msg)
    
    def section(self, title: str):
        """Log a section header."""
        self(f"\n=== {title} ===")
    
    def subsection(self, title: str):
        """Log a subsection header."""
        self(f"\n--- {title} ---")
    
    def blank(self):
        """Log an empty line."""
        self._lines.append("")
    
    def indent(self, msg: str, level: int = 1):
        """Log an indented message."""
        prefix = "  " * level
        self(f"{prefix}{msg}")
    
    def warning(self, msg: str):
        """Log a warning."""
        self(f"  WARNING: {msg}")
    
    def error(self, msg: str):
        """Log an error."""
        self(f"  ERROR: {msg}")
    
    @property
    def lines(self) -> List[str]:
        """Get all log lines."""
        return list(self._lines)
    
    def clear(self):
        """Clear all log lines."""
        self._lines.clear()
    
    def save_markdown(self, path: str, title: str = "Log"):
        """Save log as Markdown file."""
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f"# {title}\n\n")
            f.write(f"**Generated:** {datetime.now().isoformat()}\n\n")
            f.write("```\n")
            f.write("\n".join(self._lines))
            f.write("\n```\n")
    
    def save_text(self, path: str):
        """Save log as plain text."""
        with open(path, 'w', encoding='utf-8') as f:
            f.write("\n".join(self._lines))
            f.write("\n")
    
    def get_text(self) -> str:
        """Get full log as string."""
        return "\n".join(self._lines)
