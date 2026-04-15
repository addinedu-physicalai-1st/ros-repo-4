#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import urljoin, urlparse

import requests
from lxml import etree

AC_NS = "http://atlassian.com/content"
RI_NS = "http://atlassian.com/resource/identifier"
NSMAP = {"ac": AC_NS, "ri": RI_NS}

SUPPORTED_CONTAINER_TAGS = {
    "div",
    "span",
    "section",
    "article",
    "header",
    "footer",
    "tbody",
    "thead",
    "tfoot",
    "colgroup",
}
SUPPORTED_LAYOUT_TAGS = {"layout", "layout-section", "layout-cell"}
BLOCK_TAGS = {
    "p",
    "ul",
    "ol",
    "table",
    "pre",
    "blockquote",
    "hr",
    "h1",
    "h2",
    "h3",
    "h4",
    "h5",
    "h6",
}


class ConfigError(Exception):
    pass


class FatalConfluenceError(Exception):
    pass


class PageProcessingError(Exception):
    pass


@dataclass
class Config:
    base_url: str
    titles: list[str]
    output_dir: Path
    space_key: str | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export Confluence pages to Markdown files.",
    )
    parser.add_argument(
        "--config",
        required=True,
        help="Path to a JSON config file, or '-' to read from stdin.",
    )
    return parser.parse_args()


def load_config(config_arg: str) -> Config:
    if config_arg == "-":
        raw = sys.stdin.read()
    else:
        raw = Path(config_arg).read_text(encoding="utf-8")

    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ConfigError(f"invalid JSON config: {exc}") from exc

    if not isinstance(data, dict):
        raise ConfigError("config must be a JSON object")

    base_url = data.get("baseUrl")
    titles = data.get("titles")
    output_dir = data.get("outputDir")
    space_key = data.get("spaceKey")

    if not isinstance(base_url, str) or not base_url.strip():
        raise ConfigError("baseUrl must be a non-empty string")
    if not isinstance(output_dir, str) or not output_dir.strip():
        raise ConfigError("outputDir must be a non-empty string")
    if not isinstance(titles, list) or not titles:
        raise ConfigError("titles must be a non-empty array")
    if not all(isinstance(title, str) and title.strip() for title in titles):
        raise ConfigError("each title must be a non-empty string")
    if space_key is not None and (not isinstance(space_key, str) or not space_key.strip()):
        raise ConfigError("spaceKey must be a non-empty string when provided")

    normalized_base = base_url.rstrip("/")
    parsed = urlparse(normalized_base)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ConfigError("baseUrl must be a valid http(s) URL")

    resolved_output = Path(output_dir).expanduser()
    if not resolved_output.is_absolute():
        resolved_output = Path.cwd() / resolved_output

    return Config(
        base_url=normalized_base,
        titles=[title.strip() for title in titles],
        output_dir=resolved_output,
        space_key=space_key.strip() if isinstance(space_key, str) else None,
    )


def require_credentials() -> tuple[str, str]:
    email = os.environ.get("CONFLUENCE_EMAIL", "").strip()
    token = os.environ.get("CONFLUENCE_API_TOKEN", "").strip()
    if not email or not token:
        raise ConfigError("CONFLUENCE_EMAIL and CONFLUENCE_API_TOKEN must be set")
    return email, token


def cql_escape(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def cql_string_literal(value: str) -> str:
    return f'"{cql_escape(value)}"'


def cql_exact_title(value: str) -> str:
    return cql_string_literal(value)


def cql_fuzzy_title(value: str) -> str:
    return cql_string_literal(value)


def slugify(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.lower())
    slug = re.sub(r"-{2,}", "-", slug).strip("-")
    return slug


def parse_confluence_datetime(value: str | None) -> datetime:
    if not value:
        return datetime.min.replace(tzinfo=timezone.utc)
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def local_name(node: etree._Element) -> str:
    return etree.QName(node).localname


def namespace_uri(node: etree._Element) -> str | None:
    return etree.QName(node).namespace


def namespaced_attr(node: etree._Element, namespace: str, attr_name: str) -> str | None:
    return node.get(f"{{{namespace}}}{attr_name}")


def site_root(base_url: str) -> str:
    parsed = urlparse(base_url)
    return f"{parsed.scheme}://{parsed.netloc}"


def build_page_url(base_url: str, page: dict[str, Any]) -> str:
    links = page.get("_links") or {}
    webui = links.get("webui")
    if isinstance(webui, str) and webui:
        return urljoin(site_root(base_url), webui)
    return base_url


def debug_enabled() -> bool:
    value = os.environ.get("CONFLUENCE_DEBUG", "").strip().lower()
    return value in {"1", "true", "yes", "on"}


def debug_log(message: str) -> None:
    if debug_enabled():
        print(f"[confluence-debug] {message}", file=sys.stderr)


class ConfluenceClient:
    def __init__(self, base_url: str, email: str, token: str) -> None:
        self.base_url = base_url
        self.session = requests.Session()
        self.session.auth = (email, token)
        self.session.headers.update({"Accept": "application/json"})

    def _get_json(self, path: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        prepared = requests.Request("GET", url, params=params).prepare()
        debug_log(f"GET {prepared.url}")
        try:
            response = self.session.get(url, params=params, timeout=30)
        except requests.RequestException as exc:
            raise PageProcessingError(f"request failed for {url}: {exc}") from exc
        if response.status_code in {401, 403}:
            raise FatalConfluenceError(
                f"authentication failed with status {response.status_code} for {url}"
            )
        try:
            response.raise_for_status()
        except requests.HTTPError as exc:
            details = response.text.strip()
            if details:
                raise PageProcessingError(
                    f"request failed for {url}: {exc}; response body: {details}"
                ) from exc
            raise PageProcessingError(f"request failed for {url}: {exc}") from exc
        try:
            data = response.json()
        except ValueError as exc:
            raise PageProcessingError(f"invalid JSON response for {url}: {exc}") from exc
        if not isinstance(data, dict):
            raise PageProcessingError(f"unexpected response payload for {url}")
        return data

    def search_pages(self, title: str, space_key: str | None) -> list[dict[str, Any]]:
        title_clause = (
            f"title ~ {cql_fuzzy_title(title)}"
            if '"' in title
            else f"title = {cql_exact_title(title)}"
        )
        cql_parts = [
            "type = page",
            title_clause,
        ]
        if space_key:
            cql_parts.append(f"space = {cql_string_literal(space_key)}")
        debug_log(f"search cql: {' AND '.join(cql_parts)}")
        payload = self._get_json(
            "/rest/api/content/search",
            params={"cql": " AND ".join(cql_parts), "limit": 100},
        )
        results = payload.get("results", [])
        if not isinstance(results, list):
            raise PageProcessingError("unexpected search response: results is not a list")
        return [item for item in results if isinstance(item, dict)]

    def fetch_page(self, page_id: str) -> dict[str, Any]:
        return self._get_json(
            f"/rest/api/content/{page_id}",
            params={"expand": "body.storage,version"},
        )


class StorageToMarkdownConverter:
    def convert(self, storage_value: str) -> str:
        wrapped = (
            f'<root xmlns:ac="{AC_NS}" xmlns:ri="{RI_NS}">{storage_value}</root>'
        )
        try:
            root = etree.fromstring(wrapped.encode("utf-8"))
        except etree.XMLSyntaxError as exc:
            raise PageProcessingError(f"invalid body.storage XML: {exc}") from exc

        blocks = self._convert_children(root)
        markdown = "\n\n".join(block for block in blocks if block.strip())
        markdown = re.sub(r"\n{3,}", "\n\n", markdown).strip()
        return f"{markdown}\n" if markdown else ""

    def _convert_children(self, parent: etree._Element) -> list[str]:
        blocks: list[str] = []
        if parent.text and parent.text.strip():
            blocks.append(self._clean_inline_text(parent.text))

        for child in parent:
            blocks.extend(self._convert_node(child))
            if child.tail and child.tail.strip():
                blocks.append(self._clean_inline_text(child.tail))

        return [block for block in blocks if block.strip()]

    def _convert_node(self, node: etree._Element, list_indent: int = 0) -> list[str]:
        name = local_name(node)
        namespace = namespace_uri(node)

        if namespace == AC_NS and name in SUPPORTED_LAYOUT_TAGS:
            return self._convert_children(node)

        if name in SUPPORTED_CONTAINER_TAGS:
            return self._convert_children(node)

        if name in {"h1", "h2", "h3", "h4", "h5", "h6"}:
            level = int(name[1])
            content = self._render_inline_children(node).strip()
            return [f"{'#' * level} {content}"] if content else []

        if name == "p":
            content = self._render_inline_children(node).strip()
            return [content] if content else []

        if name == "blockquote":
            content = self._render_inline_children(node).strip()
            if not content:
                nested = "\n\n".join(self._convert_children(node))
                content = nested.strip()
            if not content:
                return []
            return ["\n".join(f"> {line}" if line else ">" for line in content.splitlines())]

        if name == "pre":
            code = "".join(node.itertext()).strip("\n")
            if not code:
                return []
            return [self._fenced_code_block(code, None)]

        if name in {"ul", "ol"}:
            return [self._render_list(node, ordered=(name == "ol"), indent=list_indent)]

        if name == "table":
            table = self._render_table(node)
            return [table] if table else []

        if name == "hr":
            return ["---"]

        if namespace == AC_NS and name == "structured-macro":
            macro = namespaced_attr(node, AC_NS, "name") or ""
            if macro == "code":
                code = self._macro_code_body(node)
                language = self._macro_parameter(node, "language")
                return [self._fenced_code_block(code, language)]
            return ["<!-- unsupported macro -->"]

        if namespace == AC_NS and name in {"plain-text-body", "link-body"}:
            text = "".join(node.itertext()).strip()
            return [text] if text else []

        if name in {"ac:link", "a"}:
            content = self._render_inline(node).strip()
            return [content] if content else []

        content = self._render_inline_children(node).strip()
        return [content] if content else []

    def _render_list(self, node: etree._Element, ordered: bool, indent: int) -> str:
        lines: list[str] = []
        counter = 1
        for child in node:
            if local_name(child) != "li":
                continue
            marker = f"{counter}. " if ordered else "- "
            prefix = " " * indent
            inline = self._render_list_item_inline(child)
            lines.append(f"{prefix}{marker}{inline}".rstrip())
            nested_blocks = self._render_nested_lists(child, indent + 2)
            if nested_blocks:
                lines.extend(nested_blocks)
            counter += 1
        return "\n".join(line for line in lines if line.strip())

    def _render_list_item_inline(self, node: etree._Element) -> str:
        parts: list[str] = []
        if node.text:
            parts.append(node.text)
        for child in node:
            child_name = local_name(child)
            child_ns = namespace_uri(child)
            if child_name in {"ul", "ol"}:
                continue
            if child_name == "p":
                paragraph = self._render_inline_children(child)
                if paragraph:
                    parts.append(paragraph)
            elif child_ns == AC_NS and child_name == "structured-macro":
                if (namespaced_attr(child, AC_NS, "name") or "") == "code":
                    parts.append("[code block below]")
                else:
                    parts.append("unsupported macro")
            else:
                parts.append(self._render_inline(child))
            if child.tail:
                parts.append(child.tail)
        inline = self._clean_inline_text(" ".join(parts))
        return inline or ""

    def _render_nested_lists(self, node: etree._Element, indent: int) -> list[str]:
        nested: list[str] = []
        for child in node:
            if local_name(child) in {"ul", "ol"}:
                nested.append(self._render_list(child, ordered=(local_name(child) == "ol"), indent=indent))
        return [block for block in nested if block.strip()]

    def _render_table(self, node: etree._Element) -> str:
        rows: list[list[str]] = []
        first_row_has_header = False
        for row in node.xpath(".//*[local-name()='tr']"):
            rendered_row: list[str] = []
            header_flags: list[bool] = []
            for cell in row:
                name = local_name(cell)
                if name not in {"th", "td"}:
                    continue
                rendered_row.append(self._escape_table_cell(self._render_inline_children(cell)))
                header_flags.append(name == "th")
            if rendered_row:
                if not rows:
                    first_row_has_header = any(header_flags)
                rows.append(rendered_row)

        if not rows:
            return ""

        column_count = max(len(row) for row in rows)
        normalized = [row + [""] * (column_count - len(row)) for row in rows]

        if first_row_has_header:
            header = normalized[0]
            body = normalized[1:]
        else:
            header = normalized[0]
            body = normalized[1:]

        separator = ["---"] * column_count
        lines = [
            f"| {' | '.join(header)} |",
            f"| {' | '.join(separator)} |",
        ]
        for row in body:
            lines.append(f"| {' | '.join(row)} |")
        return "\n".join(lines)

    def _escape_table_cell(self, value: str) -> str:
        cleaned = self._clean_inline_text(value)
        return cleaned.replace("|", r"\|") if cleaned else ""

    def _render_inline_children(self, node: etree._Element) -> str:
        parts: list[str] = []
        if node.text:
            parts.append(node.text)
        for child in node:
            parts.append(self._render_inline(child))
            if child.tail:
                parts.append(child.tail)
        return self._clean_inline_text("".join(parts))

    def _render_inline(self, node: etree._Element) -> str:
        name = local_name(node)
        namespace = namespace_uri(node)

        if name == "br":
            return "  \n"

        if name in {"strong", "b"}:
            content = self._render_inline_children(node)
            return f"**{content}**" if content else ""

        if name in {"em", "i"}:
            content = self._render_inline_children(node)
            return f"*{content}*" if content else ""

        if name == "code":
            content = "".join(node.itertext()).strip()
            return f"`{content}`" if content else ""

        if name == "a":
            href = node.get("href", "").strip()
            text = self._render_inline_children(node) or href
            if href:
                return f"[{text}]({href})"
            return text

        if namespace == AC_NS and name == "link":
            return self._render_confluence_link(node)

        if namespace == AC_NS and name in {"plain-text-link-body", "link-body"}:
            return self._render_inline_children(node)

        if name in SUPPORTED_CONTAINER_TAGS:
            return self._render_inline_children(node)

        return self._render_inline_children(node)

    def _render_confluence_link(self, node: etree._Element) -> str:
        link_text = self._link_text(node)
        target = self._link_target(node)
        if target:
            return f"[{link_text or target}]({target})"
        return link_text

    def _link_text(self, node: etree._Element) -> str:
        plain = node.find("ac:plain-text-link-body", namespaces=NSMAP)
        if plain is not None:
            return "".join(plain.itertext()).strip()
        rich = node.find("ac:link-body", namespaces=NSMAP)
        if rich is not None:
            return self._render_inline_children(rich).strip()
        page = node.find("ri:page", namespaces=NSMAP)
        if page is not None:
            return (
                namespaced_attr(page, RI_NS, "content-title")
                or namespaced_attr(page, RI_NS, "page-title")
                or ""
            )
        url_node = node.find("ri:url", namespaces=NSMAP)
        if url_node is not None:
            return (namespaced_attr(url_node, RI_NS, "value") or "").strip()
        attachment = node.find("ri:attachment", namespaces=NSMAP)
        if attachment is not None:
            return namespaced_attr(attachment, RI_NS, "filename") or "attachment"
        return self._render_inline_children(node).strip()

    def _link_target(self, node: etree._Element) -> str:
        url_node = node.find("ri:url", namespaces=NSMAP)
        if url_node is not None:
            return (namespaced_attr(url_node, RI_NS, "value") or "").strip()
        return ""

    def _macro_parameter(self, node: etree._Element, name: str) -> str | None:
        for parameter in node.findall("ac:parameter", namespaces=NSMAP):
            if namespaced_attr(parameter, AC_NS, "name") == name:
                value = "".join(parameter.itertext()).strip()
                return value or None
        return None

    def _macro_code_body(self, node: etree._Element) -> str:
        plain = node.find("ac:plain-text-body", namespaces=NSMAP)
        if plain is not None:
            return "".join(plain.itertext()).strip("\n")
        rich = node.find("ac:rich-text-body", namespaces=NSMAP)
        if rich is not None:
            return "\n\n".join(self._convert_children(rich)).strip()
        return ""

    def _fenced_code_block(self, code: str, language: str | None) -> str:
        language_suffix = language.strip() if language else ""
        return f"```{language_suffix}\n{code}\n```".rstrip()

    def _clean_inline_text(self, value: str) -> str:
        value = value.replace("\xa0", " ")
        value = re.sub(r"[ \t]+\n", "\n", value)
        value = re.sub(r"\n[ \t]+", "\n", value)
        placeholder = "__CONFLUENCE_BR__"
        value = value.replace("  \n", placeholder)
        value = re.sub(r"[ \t]+", " ", value)
        value = value.replace(placeholder, "  \n")
        return value.strip()


def choose_page(title: str, pages: list[dict[str, Any]]) -> dict[str, Any]:
    exact_matches = [
        page
        for page in pages
        if isinstance(page.get("title"), str) and page["title"].strip() == title.strip()
    ]
    if not exact_matches:
        raise PageProcessingError(f"no exact title match found for '{title}'")
    return max(
        exact_matches,
        key=lambda page: (
            parse_confluence_datetime((page.get("version") or {}).get("when")),
            int((page.get("version") or {}).get("number") or 0),
        ),
    )


def page_frontmatter(
    title: str,
    page_id: str,
    version: int,
    source: str,
    markdown_body: str,
) -> str:
    frontmatter = (
        "---\n"
        f'title: {json.dumps(title, ensure_ascii=False)}\n'
        f'confluence_page_id: {json.dumps(page_id)}\n'
        f"version: {version}\n"
        f'source: {json.dumps(source)}\n'
        "---\n\n"
    )
    return frontmatter + markdown_body.lstrip()


def process_title(
    client: ConfluenceClient,
    converter: StorageToMarkdownConverter,
    config: Config,
    title: str,
) -> dict[str, Any]:
    pages = client.search_pages(title, config.space_key)
    selected = choose_page(title, pages)
    page_id = str(selected.get("id") or "")
    if not page_id:
        raise PageProcessingError(f"missing page id for '{title}'")

    page = client.fetch_page(page_id)
    storage = (((page.get("body") or {}).get("storage") or {}).get("value")) or ""
    if not isinstance(storage, str):
        raise PageProcessingError(f"missing body.storage for '{title}'")

    version_number = int(((page.get("version") or {}).get("number")) or 0)
    markdown = converter.convert(storage)
    output_name = slugify(title) or f"page-{page_id}"
    output_path = config.output_dir / f"{output_name}.md"
    source = build_page_url(config.base_url, page)
    content = page_frontmatter(title, page_id, version_number, source, markdown)
    output_path.write_text(content, encoding="utf-8")

    return {
        "title": title,
        "status": "succeeded",
        "pageId": page_id,
        "version": version_number,
        "path": str(output_path),
        "source": source,
    }


def build_summary(results: list[dict[str, Any]]) -> dict[str, Any]:
    succeeded = sum(1 for result in results if result["status"] == "succeeded")
    failed = sum(1 for result in results if result["status"] == "failed")
    return {
        "processed": len(results),
        "succeeded": succeeded,
        "failed": failed,
        "results": results,
    }


def main() -> int:
    try:
        args = parse_args()
        config = load_config(args.config)
        email, token = require_credentials()
    except (ConfigError, OSError) as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1

    config.output_dir.mkdir(parents=True, exist_ok=True)

    client = ConfluenceClient(config.base_url, email, token)
    converter = StorageToMarkdownConverter()
    results: list[dict[str, Any]] = []

    try:
        for title in config.titles:
            try:
                results.append(process_title(client, converter, config, title))
            except PageProcessingError as exc:
                results.append(
                    {
                        "title": title,
                        "status": "failed",
                        "reason": str(exc),
                    }
                )
    except FatalConfluenceError as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1
    except OSError as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1

    summary = build_summary(results)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["failed"] == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
