"""核验本地Taskmaster规划的结构、链接、依赖和来源。"""

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, str]]:
    """按UTF-8读取CSV真源。"""
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    """验证规划；--planning同时保证未误标产品实施完成。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--planning", action="store_true")
    args = parser.parse_args()
    epic = Path(__file__).resolve().parents[1]
    repo = epic.parents[1]
    rows = read_rows(epic / "SUBTASKS.csv")
    expected = {"P0", "P1", "P2", "P3", "P4a", "P4b", "P5", "P6", "P7"}
    by_id = {row["id"]: row for row in rows}
    assert set(by_id) == expected and len(rows) == 9, "子任务集合/ID重复"
    deps = {key: set(filter(None, row["depends_on"].split(";"))) for key, row in by_id.items()}
    finished: set[str] = set()
    active: set[str] = set()

    def visit(key: str) -> None:
        assert key not in active, f"循环依赖：{key}"
        if key in finished:
            return
        active.add(key)
        for dependency in deps[key]:
            assert dependency in expected, f"未知依赖：{dependency}"
            visit(dependency)
        active.remove(key)
        finished.add(key)

    def ancestors(key: str) -> set[str]:
        result: set[str] = set()
        for dependency in deps[key]:
            result.add(dependency)
            result.update(ancestors(dependency))
        return result

    for key in by_id:
        visit(key)
    assert not {"P4b", "P7"} & ancestors("P6"), "独立扩展误成为核心前置"
    sections = ["任务与依赖", "输入与启动条件", "修改范围与责任", "必须交付的产物",
                "实现规则", "执行步骤", "验收条件", "专项验证场景", "验证命令与证据",
                "风险与阻断", "公共约束与环境"]
    steps_count = 0
    for key, row in by_id.items():
        directory = (epic / row["task_dir"]).resolve()
        assert directory.is_relative_to(epic / "tasks"), "子目录越界"
        assert row["task_type"] == "single-full"
        for name in ("SPEC.md", "TODO.csv", "PROGRESS.md"):
            assert (directory / name).is_file(), f"缺失文件：{directory / name}"
        assert (directory / "raw").is_dir(), f"缺失证据目录：{key}"
        spec = (directory / "SPEC.md").read_text(encoding="utf-8")
        for section in sections:
            assert "## " + section in spec, f"规格章节缺失：{key}/{section}"
        steps = read_rows(directory / "TODO.csv")
        ids = [step["id"] for step in steps]
        assert ids == [str(i) for i in range(1, len(steps) + 1)], f"叶子步骤ID：{key}"
        steps_count += len(steps)
        for step in steps:
            assert step["acceptance_criteria"] and step["validation_command"]
        if args.planning:
            assert row["status"] == "TODO" and not row["completed_at"]
            assert all(step["status"] == "TODO" and not step["completed_at"] for step in steps)
        if row["status"] == "DONE":
            assert all(by_id[d]["status"] == "DONE" for d in deps[key])
            assert all(step["status"] == "DONE" for step in steps)
    assert steps_count == 51, "叶子步数量变化须同步Epic"
    links = 0
    for path in epic.rglob("*.md"):
        content = path.read_text(encoding="utf-8")
        assert not re.search(r"[ \t]+$", content, re.MULTILINE), f"尾空白：{path}"
        for target in re.findall(r"\[[^\]]*\]\(([^)]+)\)", content):
            target = target.strip("<>").split("#", 1)[0]
            if not target or "://" in target:
                continue
            assert (path.parent / target).exists(), f"失效链接：{path} -> {target}"
            links += 1
    provenance = json.loads((epic / "raw/design-source.json").read_text(encoding="utf-8-sig"))
    source = repo / provenance["path"]
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert digest == provenance["sha256"], "设计真源已变化，须重新审查规划"
    print(json.dumps({"children": len(rows), "leaf_steps": steps_count, "links": links,
                      "dag": "acyclic", "source_sha256": digest,
                      "implementation": "TODO" if args.planning else "checked"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
