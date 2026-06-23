#!/usr/bin/env python3
"""
export_chat.py — convert a Claude Code .jsonl transcript into a readable Markdown file.

Runs entirely on your machine (no model tokens used). Reads the JSONL line by line,
pulls out the human <-> assistant text (and a one-line note for each tool call), and
writes a clean conversation_log.md.

Usage:
    python3 export_chat.py                      # uses the default paths below
    python3 export_chat.py INPUT.jsonl OUT.md   # or pass your own
"""
import json
import sys

DEFAULT_IN = ("/home/omar/.claude/projects/-home-omar-Desktop-Thesis/"
              "8bbb9ac5-55d0-47ab-b405-1e7e1f565b40.jsonl")
DEFAULT_OUT = ("/home/omar/Desktop/Thesisorg/docs/teaching/conversation_log.md")


def text_of(content):
    """Return (text, tool_notes) from a message 'content' field."""
    if isinstance(content, str):
        return content.strip(), []
    text_parts, tool_notes = [], []
    for block in content or []:
        if not isinstance(block, dict):
            continue
        btype = block.get("type")
        if btype == "text":
            text_parts.append(block.get("text", "").strip())
        elif btype == "tool_use":
            tool_notes.append(f"_(tool: {block.get('name', '?')})_")
        elif btype == "tool_result":
            # keep tool results out of the readable log (usually noisy output)
            pass
    return "\n\n".join(t for t in text_parts if t), tool_notes


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IN
    dst = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT

    lines_out = ["# Conversation Log\n",
                 f"_Exported from `{src}`_\n"]
    n_user = n_asst = 0

    with open(src, "r", encoding="utf-8") as f:
        for raw in f:
            raw = raw.strip()
            if not raw:
                continue
            try:
                entry = json.loads(raw)
            except json.JSONDecodeError:
                continue

            etype = entry.get("type")
            msg = entry.get("message") or {}
            role = msg.get("role") or etype
            if role not in ("user", "assistant"):
                continue

            text, tool_notes = text_of(msg.get("content"))
            if not text and not tool_notes:
                continue

            if role == "user":
                # skip pure tool-result turns (no human text)
                if not text:
                    continue
                n_user += 1
                lines_out.append(f"\n## 🧑 You\n\n{text}\n")
            else:
                n_asst += 1
                body = text if text else " ".join(tool_notes)
                if text and tool_notes:
                    body = text + "\n\n" + " ".join(tool_notes)
                lines_out.append(f"\n## 🤖 Claude\n\n{body}\n")

    with open(dst, "w", encoding="utf-8") as f:
        f.write("\n".join(lines_out))

    print(f"Wrote {dst}")
    print(f"  {n_user} of your messages, {n_asst} assistant messages")


if __name__ == "__main__":
    main()
