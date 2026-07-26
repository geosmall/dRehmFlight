#!/usr/bin/env python3
"""
Generate WinMerge-style side-by-side HTML diff report.
Compares two files and produces a visually similar output to WinMerge.
"""

import difflib
import html
import sys
import re
from pathlib import Path

# WinMerge-style colors
COLORS = {
    'unchanged': '#ffffff',
    'changed': '#efcb05',      # Yellow for modifications
    'added': '#a0ffa0',        # Light green for additions
    'deleted': '#ffa0a0',      # Light red for deletions
    'missing': '#c0c0c0',      # Gray for missing lines
    'header': 'linear-gradient(mediumblue, darkblue)',
}

def syntax_highlight(text):
    """Apply simple C/C++ syntax highlighting using token-based approach."""
    # Escape HTML first
    escaped = html.escape(text)

    # If line is empty or whitespace, return as-is
    if not escaped.strip():
        return escaped

    result = []
    i = 0
    n = len(escaped)

    while i < n:
        # Check for // comment (to end of line)
        if escaped[i:i+2] == '//':
            result.append(f'<span style="color:#008000;">{escaped[i:]}</span>')
            break

        # Check for /* comment start
        if escaped[i:i+2] == '/*':
            end = escaped.find('*/', i+2)
            if end == -1:
                result.append(f'<span style="color:#008000;">{escaped[i:]}</span>')
                break
            else:
                result.append(f'<span style="color:#008000;">{escaped[i:end+2]}</span>')
                i = end + 2
                continue

        # Check for preprocessor directive
        if escaped[i] == '#' and (i == 0 or escaped[i-1].isspace() or result == []):
            # Find end of directive word
            j = i + 1
            while j < n and (escaped[j].isalnum() or escaped[j] == '_'):
                j += 1
            result.append(f'<span style="color:#0000ff;font-weight:bold;">{escaped[i:j]}</span>')
            i = j
            continue

        # Check for string literal
        if escaped[i] == '&' and escaped[i:i+6] == '&quot;':
            # Find closing quote
            j = i + 6
            while j < n:
                if escaped[j:j+6] == '&quot;':
                    j += 6
                    break
                j += 1
            result.append(f'<span style="color:#800080;">{escaped[i:j]}</span>')
            i = j
            continue

        # Check for char literal
        if escaped[i] == '&' and escaped[i:i+6] == '&#x27;':
            j = i + 6
            while j < n:
                if escaped[j:j+6] == '&#x27;':
                    j += 6
                    break
                j += 1
            result.append(f'<span style="color:#800080;">{escaped[i:j]}</span>')
            i = j
            continue

        # Regular character
        result.append(escaped[i])
        i += 1

    return ''.join(result)

def generate_winmerge_diff(file1_path, file2_path, output_path):
    """Generate WinMerge-style HTML diff report."""

    # Read files
    with open(file1_path, 'r', encoding='utf-8', errors='replace') as f:
        file1_lines = f.readlines()
    with open(file2_path, 'r', encoding='utf-8', errors='replace') as f:
        file2_lines = f.readlines()

    # Strip trailing newlines for comparison
    file1_lines = [line.rstrip('\n\r') for line in file1_lines]
    file2_lines = [line.rstrip('\n\r') for line in file2_lines]

    # Get diff using SequenceMatcher for better alignment
    matcher = difflib.SequenceMatcher(None, file1_lines, file2_lines)

    html_rows = []
    line1_num = 0
    line2_num = 0
    diff_count = 0

    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == 'equal':
            # Unchanged lines
            for i in range(i2 - i1):
                line1 = file1_lines[i1 + i]
                line2 = file2_lines[j1 + i]
                line1_num += 1
                line2_num += 1

                content1 = syntax_highlight(line1) if line1.strip() else '&nbsp;'
                content2 = syntax_highlight(line2) if line2.strip() else '&nbsp;'

                html_rows.append(f'''<tr>
<td class="ln">{line1_num}</td><td class="code" style="background-color:{COLORS['unchanged']};"><code>{content1}</code></td>
<td class="ln">{line2_num}</td><td class="code" style="background-color:{COLORS['unchanged']};"><code>{content2}</code></td>
</tr>''')

        elif tag == 'replace':
            # Changed lines - show side by side
            diff_count += 1
            max_len = max(i2 - i1, j2 - j1)
            for i in range(max_len):
                if i < (i2 - i1):
                    line1 = file1_lines[i1 + i]
                    line1_num += 1
                    ln1 = f'<a id="d{diff_count}" href="#d{diff_count}">{line1_num}</a>'
                    content1 = syntax_highlight(line1) if line1.strip() else '&nbsp;'
                    bg1 = COLORS['changed']
                else:
                    ln1 = ''
                    content1 = '&nbsp;'
                    bg1 = COLORS['missing']

                if i < (j2 - j1):
                    line2 = file2_lines[j1 + i]
                    line2_num += 1
                    ln2 = str(line2_num)
                    content2 = syntax_highlight(line2) if line2.strip() else '&nbsp;'
                    bg2 = COLORS['changed']
                else:
                    ln2 = ''
                    content2 = '&nbsp;'
                    bg2 = COLORS['missing']

                html_rows.append(f'''<tr>
<td class="ln">{ln1}</td><td class="code" style="background-color:{bg1};"><code>{content1}</code></td>
<td class="ln">{ln2}</td><td class="code" style="background-color:{bg2};"><code>{content2}</code></td>
</tr>''')

        elif tag == 'delete':
            # Lines only in file1 (deleted from file2's perspective)
            diff_count += 1
            for i in range(i2 - i1):
                line1 = file1_lines[i1 + i]
                line1_num += 1
                ln1 = f'<a id="d{diff_count}" href="#d{diff_count}">{line1_num}</a>'
                content1 = syntax_highlight(line1) if line1.strip() else '&nbsp;'

                html_rows.append(f'''<tr>
<td class="ln">{ln1}</td><td class="code" style="background-color:{COLORS['deleted']};"><code>{content1}</code></td>
<td class="ln"></td><td class="code" style="background-color:{COLORS['missing']};"><code>&nbsp;</code></td>
</tr>''')

        elif tag == 'insert':
            # Lines only in file2 (added)
            diff_count += 1
            for i in range(j2 - j1):
                line2 = file2_lines[j1 + i]
                line2_num += 1
                ln2 = f'<a id="d{diff_count}" href="#d{diff_count}">{line2_num}</a>'
                content2 = syntax_highlight(line2) if line2.strip() else '&nbsp;'

                html_rows.append(f'''<tr>
<td class="ln"></td><td class="code" style="background-color:{COLORS['missing']};"><code>&nbsp;</code></td>
<td class="ln">{ln2}</td><td class="code" style="background-color:{COLORS['added']};"><code>{content2}</code></td>
</tr>''')

    # Build complete HTML
    html_content = f'''<!DOCTYPE html>
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
<title>File Compare Report</title>
<style>
* {{ box-sizing: border-box; }}
table {{ table-layout: fixed; width: 100%; margin: 0; border: 1px solid #a0a0a0; box-shadow: 1px 1px 2px rgba(0,0,0,0.15); border-collapse: collapse; }}
colgroup col.ln {{ width: 45px; }}
colgroup col.code {{ width: calc(50% - 45px); }}
th {{ position: sticky; top: 0; z-index: 1; }}
td, th {{ font-size: 11pt; padding: 0 3px; border: none; }}
tr {{ vertical-align: top; }}
.title {{ font-weight: bold; color: white; background: {COLORS['header']}; text-align: center; padding: 4px; }}
.ln {{ text-align: right; color: #606060; background-color: #f0f0f0; border-right: 1px solid #c0c0c0; font-family: monospace; font-size: 10pt; padding-right: 5px; }}
.ln a {{ color: #606060; text-decoration: none; }}
.ln a:hover {{ color: #0000ff; }}
.code {{ font-family: Consolas, Monaco, "Courier New", monospace; font-size: 10pt; white-space: pre; overflow: hidden; text-overflow: ellipsis; }}
code {{ font-family: inherit; }}
</style>
</head>
<body>
<div style="font-size:10pt; color:#606060; margin-bottom:10px; font-family: sans-serif;">
<strong>Legend:</strong>
<span style="background-color:{COLORS['changed']}; padding:2px 8px; margin-left:10px;">Modified</span>
<span style="background-color:{COLORS['deleted']}; padding:2px 8px; margin-left:5px;">Left only</span>
<span style="background-color:{COLORS['added']}; padding:2px 8px; margin-left:5px;">Right only</span>
<span style="background-color:{COLORS['missing']}; padding:2px 8px; margin-left:5px;">No content</span>
</div>
<table>
<colgroup>
<col class="ln"><col class="code">
<col class="ln"><col class="code">
</colgroup>
<thead>
<tr>
<th colspan="2" class="title">{html.escape(str(Path(file1_path).name))}</th>
<th colspan="2" class="title">{html.escape(str(Path(file2_path).name))}</th>
</tr>
</thead>
<tbody>
{''.join(html_rows)}
</tbody>
</table>
<p style="font-size:10pt; color:#606060; margin-top:10px; font-family: sans-serif;">
Left: {len(file1_lines)} lines | Right: {len(file2_lines)} lines | {diff_count} difference regions
</p>
</body>
</html>'''

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_content)

    print(f"Generated: {output_path}")
    print(f"  Left file:  {len(file1_lines)} lines")
    print(f"  Right file: {len(file2_lines)} lines")
    print(f"  Difference regions: {diff_count}")

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <file1> <file2> <output.html>")
        sys.exit(1)

    generate_winmerge_diff(sys.argv[1], sys.argv[2], sys.argv[3])
