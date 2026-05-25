import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.font_manager as fm
import networkx as nx

# 使用支持中文的系统字体
plt.rcParams['font.family'] = ['PingFang HK', 'Heiti TC', 'Arial Unicode MS', 'DejaVu Sans']

# ── 节点定义（id -> 显示名, 层级颜色） ──────────────────────────
LAYERS = {
    # 基础层
    "utilities.h":      0,
    "rigid_body.h":     0,
    "system_state.h":   0,
    # 基本数据结构层
    "matrix.h":         1,
    "ode_solver.h":     1,
    "force_generator.h":1,
    # 扩展层
    "sparse_matrix.h":          2,
    "euler_ode_solver.h":       2,
    "rk4_ode_solver.h":         2,
    "nsv_ode_solver.h":         2,
    "gravity_force_generator.h":2,
    "static_force_generator.h": 2,
    "spring.h":                 2,
    "constraint.h":             2,
    # SLE 基础
    "sle_solver.h":             3,
    # 具体 SLE 求解器
    "gauss_seidel_sle_solver.h":        4,
    "gaussian_elimination_sle_solver.h":4,
    "conjugate_gradient_sle_solver.h":  4,
    # 刚体系统抽象
    "rigid_body_system.h":      5,
    # 具体系统实现
    "generic_rigid_body_system.h":      6,
    "optimized_nsv_rigid_body_system.h":6,
    # 入口
    "scs.h":                    7,
}

LAYER_COLORS = {
    0: "#AED6F1",   # 浅蓝 - 基础层
    1: "#A9DFBF",   # 浅绿 - 数据结构层
    2: "#F9E79F",   # 浅黄 - 扩展层
    3: "#F0B27A",   # 橙   - SLE 基础
    4: "#F1948A",   # 浅红 - 具体求解器
    5: "#BB8FCE",   # 紫   - 系统抽象
    6: "#85C1E9",   # 蓝   - 具体系统
    7: "#E74C3C",   # 红   - 入口
}

LAYER_LABELS = {
    0: "基础层",
    1: "数据结构层",
    2: "扩展层",
    3: "SLE 抽象层",
    4: "SLE 求解器层",
    5: "刚体系统抽象层",
    6: "具体系统实现层",
    7: "统一入口",
}

# ── 边定义 ────────────────────────────────────────────────────
EDGES = [
    # L1 -> ROOT
    ("matrix.h",            "utilities.h"),
    ("ode_solver.h",        "system_state.h"),
    ("force_generator.h",   "system_state.h"),
    # L2
    ("sparse_matrix.h",             "matrix.h"),
    ("sparse_matrix.h",             "utilities.h"),
    ("euler_ode_solver.h",          "ode_solver.h"),
    ("rk4_ode_solver.h",            "ode_solver.h"),
    ("nsv_ode_solver.h",            "ode_solver.h"),
    ("gravity_force_generator.h",   "force_generator.h"),
    ("gravity_force_generator.h",   "rigid_body.h"),
    ("static_force_generator.h",    "force_generator.h"),
    ("static_force_generator.h",    "rigid_body.h"),
    ("spring.h",                    "force_generator.h"),
    ("spring.h",                    "rigid_body.h"),
    ("constraint.h",                "system_state.h"),
    ("constraint.h",                "rigid_body.h"),
    ("constraint.h",                "matrix.h"),
    ("constraint.h",                "utilities.h"),
    # L3
    ("sle_solver.h",    "matrix.h"),
    ("sle_solver.h",    "sparse_matrix.h"),
    # L4
    ("gauss_seidel_sle_solver.h",           "sle_solver.h"),
    ("gaussian_elimination_sle_solver.h",   "sle_solver.h"),
    ("gaussian_elimination_sle_solver.h",   "utilities.h"),
    ("conjugate_gradient_sle_solver.h",     "sle_solver.h"),
    # L5
    ("rigid_body_system.h", "rigid_body.h"),
    ("rigid_body_system.h", "constraint.h"),
    ("rigid_body_system.h", "force_generator.h"),
    ("rigid_body_system.h", "matrix.h"),
    ("rigid_body_system.h", "sparse_matrix.h"),
    ("rigid_body_system.h", "system_state.h"),
    # L6
    ("generic_rigid_body_system.h",         "rigid_body_system.h"),
    ("generic_rigid_body_system.h",         "sle_solver.h"),
    ("generic_rigid_body_system.h",         "ode_solver.h"),
    ("optimized_nsv_rigid_body_system.h",   "rigid_body_system.h"),
    ("optimized_nsv_rigid_body_system.h",   "sle_solver.h"),
    ("optimized_nsv_rigid_body_system.h",   "nsv_ode_solver.h"),
    # 入口
    ("scs.h", "generic_rigid_body_system.h"),
    ("scs.h", "optimized_nsv_rigid_body_system.h"),
    ("scs.h", "euler_ode_solver.h"),
    ("scs.h", "rk4_ode_solver.h"),
    ("scs.h", "nsv_ode_solver.h"),
    ("scs.h", "gauss_seidel_sle_solver.h"),
    ("scs.h", "gaussian_elimination_sle_solver.h"),
    ("scs.h", "conjugate_gradient_sle_solver.h"),
    ("scs.h", "static_force_generator.h"),
    ("scs.h", "gravity_force_generator.h"),
    ("scs.h", "spring.h"),
]

# ── 构建图 ────────────────────────────────────────────────────
G = nx.DiGraph()
for node in LAYERS:
    G.add_node(node)
for u, v in EDGES:
    G.add_edge(u, v)

# ── 手动分层布局 ──────────────────────────────────────────────
# 每层节点按水平均匀排列，层号越大 y 越高
layer_nodes = {}
for node, layer in LAYERS.items():
    layer_nodes.setdefault(layer, []).append(node)

# 固定 x 间距，层间 y 间距
X_SPACING = 3.5
Y_SPACING = 2.6

pos = {}
# 手动调整某些层的 x 间距，避免节点重叠
LAYER_X_SPACING = {0: 5.5, 1: 5.5, 2: 3.2, 3: 5.5, 4: 3.5, 5: 5.5, 6: 5.5, 7: 5.5}
for layer, nodes in layer_nodes.items():
    n = len(nodes)
    sp = LAYER_X_SPACING.get(layer, X_SPACING)
    xs = [(i - (n - 1) / 2) * sp for i in range(n)]
    y = layer * Y_SPACING
    for node, x in zip(nodes, xs):
        pos[node] = (x, y)

# ── 绘图 ──────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(28, 26))
ax.set_facecolor("#F8F9FA")
fig.patch.set_facecolor("#F8F9FA")

node_colors = [LAYER_COLORS[LAYERS[n]] for n in G.nodes()]

nx.draw_networkx_nodes(
    G, pos, ax=ax,
    node_color=node_colors,
    node_size=5500,
    node_shape="s",
    alpha=0.92,
)
nx.draw_networkx_labels(
    G, pos, ax=ax,
    font_size=7.5,
    font_weight="bold",
    font_family="monospace",
)
nx.draw_networkx_edges(
    G, pos, ax=ax,
    arrows=True,
    arrowsize=16,
    arrowstyle="-|>",
    edge_color="#555555",
    width=1.1,
    connectionstyle="arc3,rad=0.05",
    min_source_margin=30,
    min_target_margin=30,
)

# 图例
legend_patches = [
    mpatches.Patch(color=LAYER_COLORS[i], label=f"Layer {i}  {LAYER_LABELS[i]}")
    for i in sorted(LAYER_LABELS)
]
ax.legend(
    handles=legend_patches,
    loc="lower right",
    fontsize=8,
    framealpha=0.9,
    title="图例",
    title_fontsize=9,
)

ax.set_title("simple-2d-constraint-solver  头文件引用关系图\n（箭头方向 = 包含方向，约束文件已排除）",
             fontsize=13, fontweight="bold", pad=16)
ax.axis("off")

plt.tight_layout()
out = "/Users/laserss/Desktop/simple-2d-constraint-solver/header_dependency_graph.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
print("saved:", out)
