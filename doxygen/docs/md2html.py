# -*- coding: utf-8 -*-
"""将本目录下的 .md 文档转换为带内嵌样式的 .html。

仅依赖 Python 标准库,处理本仓库文档用到的 Markdown 语法:
标题(#/##/###/####)、引用(>)、表格(|)、代码块(```)、
行内代码(`)、无序/有序列表(- / 1.)、加粗(**)、水平线(---)、链接、段落。

用法: python md2html.py [file1.md ...]   (缺省转换本目录全部 .md)
"""

import html
import os
import re
import sys

CSS = """
:root {
  --fg: #1f2328; --bg: #ffffff; --muted: #656d76;
  --border: #d0d7de; --accent: #0969da; --code-bg: #f6f8fa;
}
* { box-sizing: border-box; }
body {
  margin: 0; color: var(--fg); background: var(--bg);
  font-family: -apple-system, "Segoe UI", "Microsoft YaHei", "PingFang SC",
               Helvetica, Arial, sans-serif;
  line-height: 1.7; font-size: 16px;
}
main { max-width: 900px; margin: 0 auto; padding: 32px 24px 96px; }
h1 { font-size: 2em; border-bottom: 1px solid var(--border); padding-bottom: .3em; }
h2 { font-size: 1.5em; border-bottom: 1px solid var(--border); padding-bottom: .3em;
     margin-top: 2em; }
h3 { font-size: 1.2em; margin-top: 1.6em; }
h4 { font-size: 1.05em; }
a { color: var(--accent); text-decoration: none; }
a:hover { text-decoration: underline; }
code { font-family: "SFMono-Regular", Consolas, "Liberation Mono", monospace;
       background: var(--code-bg); padding: .15em .4em; border-radius: 4px;
       font-size: .9em; }
pre { background: var(--code-bg); padding: 14px 16px; border-radius: 6px;
      overflow-x: auto; line-height: 1.5; }
pre code { background: none; padding: 0; font-size: .88em; }
blockquote { margin: 1em 0; padding: .4em 1em; color: var(--muted);
             border-left: 4px solid var(--border); background: #fafbfc; }
table { border-collapse: collapse; margin: 1em 0; width: 100%; display: block;
        overflow-x: auto; }
th, td { border: 1px solid var(--border); padding: 7px 12px; text-align: left; }
th { background: var(--code-bg); font-weight: 600; }
tr:nth-child(even) td { background: #fafbfc; }
hr { border: none; border-top: 1px solid var(--border); margin: 2em 0; }
ul, ol { padding-left: 1.8em; }
li { margin: .25em 0; }
.toc { background: #f6f8fa; border: 1px solid var(--border); border-radius: 6px;
       padding: 12px 20px; margin: 1.2em 0; }
.toc summary { font-weight: 600; cursor: pointer; }
footer { margin-top: 3em; color: var(--muted); font-size: .85em;
         border-top: 1px solid var(--border); padding-top: 1em; }
"""


def inline(text):
    """行内元素:链接、行内代码、加粗。"""
    codes = []

    def stash_code(m):
        codes.append(html.escape(m.group(1)))
        return f"\x00{len(codes) - 1}\x00"

    text = re.sub(r"`([^`]+)`", stash_code, text)
    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", text)
    for i, c in enumerate(codes):
        text = text.replace(f"\x00{i}\x00", f"<code>{c}</code>")
    return text


def slugify(text):
    """从标题生成锚点 slug:先剥离行内 markdown,再转小写连字符。"""
    text = re.sub(r"`([^`]+)`", r"\1", text)                # 行内代码
    text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)     # 链接
    text = re.sub(r"\*\*([^*]+)\*\*", r"\1", text)           # 加粗
    return re.sub(r"[^\w一-鿿]+", "-", text).strip("-").lower()


def convert(md_text):
    lines = md_text.split("\n")
    out = []
    i = 0
    n = len(lines)

    def is_table_sep(line):
        return bool(re.match(r"^\s*\|?[\s:|-]+\|?\s*$", line)) and "|" in line and re.search(r"-", line)

    while i < n:
        line = lines[i]

        # 代码块
        if line.strip().startswith("```"):
            lang = line.strip()[3:].strip()
            buf = []
            i += 1
            while i < n and not lines[i].strip().startswith("```"):
                buf.append(lines[i])
                i += 1
            i += 1  # 跳过结束 ```
            code = html.escape("\n".join(buf))
            lang_attr = f' class="language-{html.escape(lang)}"' if lang else ""
            out.append(f"<pre><code{lang_attr}>{code}</code></pre>")
            continue

        # 标题
        m = re.match(r"^(#{1,6})\s+(.*)$", line)
        if m:
            level = len(m.group(1))
            title = m.group(2)
            if level in (2, 3):
                out.append(f'<h{level} id="{slugify(title)}">{inline(title)}</h{level}>')
            else:
                out.append(f"<h{level}>{inline(title)}</h{level}>")
            i += 1
            continue

        # 水平线
        if re.match(r"^\s*---\s*$", line):
            out.append("<hr>")
            i += 1
            continue

        # 引用
        if line.startswith(">"):
            buf = []
            while i < n and lines[i].startswith(">"):
                buf.append(lines[i][1:].strip())
                i += 1
            out.append(f"<blockquote>{inline(' '.join(buf))}</blockquote>")
            continue

        # 表格
        if "|" in line and i + 1 < n and is_table_sep(lines[i + 1]):
            header = [c.strip() for c in line.strip().strip("|").split("|")]
            i += 2  # 跳过表头 + 分隔行
            rows = []
            while i < n and "|" in lines[i] and lines[i].strip():
                rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
                i += 1
            head_html = "".join(f"<th>{inline(c)}</th>" for c in header)
            body_html = ""
            for r in rows:
                body_html += "<tr>" + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>"
            out.append(f"<table><thead><tr>{head_html}</tr></thead><tbody>{body_html}</tbody></table>")
            continue

        # 列表(无序 / 有序)
        m_ul = re.match(r"^\s*[-*]\s+(.*)$", line)
        m_ol = re.match(r"^\s*\d+\.\s+(.*)$", line)
        if m_ul or m_ol:
            tag = "ul" if m_ul else "ol"
            buf = []
            while i < n:
                cur = lines[i]
                mm = re.match(r"^\s*[-*]\s+(.*)$", cur) if tag == "ul" else re.match(r"^\s*\d+\.\s+(.*)$", cur)
                if not mm:
                    break
                buf.append(f"<li>{inline(mm.group(1))}</li>")
                i += 1
            out.append(f"<{tag}>{''.join(buf)}</{tag}>")
            continue

        # 空行
        if not line.strip():
            i += 1
            continue

        # 普通段落(合并连续非空行)
        buf = [line]
        i += 1
        while i < n and lines[i].strip() and not re.match(r"^(#{1,6}\s|```|>\s|[-*]\s|\d+\.\s|\s*---)", lines[i]) and "|" not in lines[i]:
            buf.append(lines[i])
            i += 1
        out.append(f"<p>{inline(' '.join(buf))}</p>")

    return "\n".join(out)


def build_toc(md_text):
    """从标题生成目录(仅收集 h2/h3)。"""
    items = []
    for m in re.finditer(r"^(#{2,3})\s+(.*)$", md_text, re.M):
        level = len(m.group(1))
        title = m.group(2)
        items.append((level, title, slugify(title)))
    if not items:
        return ""
    lis = []
    for level, title, slug in items:
        indent = "&nbsp;&nbsp;" * (level - 2)
        lis.append(f'<li>{indent}<a href="#{slug}">{inline(title)}</a></li>')
    return '<details class="toc" open><summary>目录</summary><ul>' + "".join(lis) + "</ul></details>"


def title_of(md_text):
    m = re.search(r"^#\s+(.*)$", md_text, re.M)
    return m.group(1) if m else "文档"


def render(md_path):
    with open(md_path, encoding="utf-8") as f:
        md_text = f.read()
    title = title_of(md_text)
    body = convert(md_text)
    toc = build_toc(md_text)
    doc = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{html.escape(title)}</title>
<style>{CSS}</style>
</head>
<body>
<main>
<h1>{html.escape(title)}</h1>
{toc}
{body}
<footer>由 md2html.py 自动生成 · StepMotorCtrl_42 项目文档</footer>
</main>
</body>
</html>
"""
    out_path = os.path.splitext(md_path)[0] + ".html"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(doc)
    return out_path


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    targets = sys.argv[1:]
    if not targets:
        targets = [f for f in os.listdir(here) if f.endswith(".md")]
    for t in targets:
        path = t if os.path.isabs(t) else os.path.join(here, t)
        if not os.path.exists(path):
            print(f"[跳过] 不存在: {t}")
            continue
        out = render(path)
        print(f"[生成] {os.path.basename(out)}")


if __name__ == "__main__":
    main()
