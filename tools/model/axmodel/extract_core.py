"""openpilot master driving_supercombo.onnx에서 NPU 코어를 잘라 낸다.

이 스크립트는 예전 출시 그래프(65a08adc)용이다. 런타임이 지금 받는 Smoked Cheddar(e170f08) 그래프는 큐가 길고
(desire 132틱, 특징 128틱) action_t가 plan에 들어가며 노드 이름도 달라, 그 axmodel은 모델 프로젝트의 빌드가 만든다.

배포 그래프에는 이력 큐(이미지/desire/특징 시프트 레지스터, desire 20->5 Hz max-pool)도
들어 있다. 이것들은 런타임이 CPU에서 하는 값싼 텐서 재배치라, 코어에는 네트워크만 남긴다:

  input_imgs      [1,12,128,256]  2 프레임 x 6 YUV 평면(road)
  big_input_imgs  [1,12,128,256]  wide도 같다
  desire          [1,25,8]        5 Hz로 max-pool한 desire 이력
  features_buffer [1,24,512]      5 Hz 특징 이력
  traffic_convention [1,2]   (action_t는 배포 그래프에서 쓰이지 않는 Cast 하나에만 들어가 뺐다.
                             새 그래프가 이를 쓰기 시작하면 스크립트가 멈춘다)
  -> outputs [1,2576]

fp16 가중치/cast는 fp32로 올린다(Pulsar2는 fp32에서 양자화한다).
"""
import argparse
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper, utils

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("src", help="driving_supercombo.onnx")
parser.add_argument("dst", help="core_fp32.onnx")
args = parser.parse_args()
src, dst = args.src, args.dst
tmp = dst + ".tmp.onnx"
cut = {"_unsafe_view": "input_imgs", "_unsafe_view_1": "big_input_imgs", "transpose": "desire",
       "transpose_1": "features_buffer", "traffic_convention": "traffic_convention"}
m = onnx.shape_inference.infer_shapes(onnx.load(src))


def reaches_outputs(graph, name):
    """텐서 `name`이 그래프 출력 중 하나로 이어지는지."""
    outputs, frontier, seen = {o.name for o in graph.output}, [name], set()
    while frontier:
        t = frontier.pop()
        if t in outputs:
            return True
        seen.add(t)
        for n in graph.node:
            if t in n.input:
                frontier.extend(o for o in n.output if o not in seen)
    return False


# 배포 그래프는 action_t(openpilot modeld가 넘기는 횡/종방향 지연)를 결과를 아무도 읽지
# 않는 Cast 하나에 넣으므로 코어는 이를 뺀다. 이를 쓰기 시작한 모델은 새 코어 입력과
# 런타임 지원이 필요하다: 여기서 멈춘다.
if any(i.name == "action_t" for i in m.graph.input) and reaches_outputs(m.graph, "action_t"):
    sys.exit("action_t now feeds the outputs: add it as a core input and feed it at runtime")
onnx.save(m, tmp)
utils.extract_model(tmp, tmp, list(cut), ["outputs"])
m = onnx.load(tmp)
g = m.graph
# fp16 -> fp32
for init in g.initializer:
    if init.data_type == TensorProto.FLOAT16:
        init.CopyFrom(numpy_helper.from_array(numpy_helper.to_array(init).astype(np.float32), init.name))
for n in g.node:
    for a in n.attribute:
        if n.op_type == "Cast" and a.name == "to" and a.i == TensorProto.FLOAT16:
            a.i = TensorProto.FLOAT
        if a.type == onnx.AttributeProto.TENSOR and a.t.data_type == TensorProto.FLOAT16:
            a.t.CopyFrom(numpy_helper.from_array(numpy_helper.to_array(a.t).astype(np.float32)))
# uint8 이미지 입력 -> fp32 그래프 입력, 런타임 이름으로 바꾼다
for i in g.input:
    i.type.tensor_type.elem_type = TensorProto.FLOAT
ren = {k: v for k, v in cut.items() if k != v}
for n in g.node:
    for j, x in enumerate(n.input):
        if x in ren: n.input[j] = ren[x]
for i in g.input:
    if i.name in ren: i.name = ren[i.name]
# 이제 모든 Cast가 fp32->fp32다(Pulsar2는 opset-20 Cast를 받지 않는다): 이들을 우회하고,
# Identity도 함께 우회한다
alias = {}
keep = []
for n in g.node:
    if n.op_type == "Cast":
        assert next(a.i for a in n.attribute if a.name == "to") == TensorProto.FLOAT
        alias[n.output[0]] = alias.get(n.input[0], n.input[0])
    elif n.op_type == "Identity":   # Pulsar2 7.0은 단독 Identity 연결을 받지 않는다
        alias[n.output[0]] = alias.get(n.input[0], n.input[0])
    else:
        keep.append(n)
for n in keep:
    for j, x in enumerate(n.input):
        n.input[j] = alias.get(x, x)
outs = {o.name for o in g.output}
for o in outs & set(alias):   # Cast가 만든 그래프 출력: 생산 노드의 출력 이름을 바꾼다
    src_name = alias[o]
    for n in keep:
        for j, x in enumerate(n.output):
            if x == src_name: n.output[j] = o
        for j, x in enumerate(n.input):
            if x == src_name: n.input[j] = o
del g.node[:]
g.node.extend(keep)

# Pulsar2는 policy의 GatherND(상수 인덱스 = 마지막 9개 특징 토큰)도
# Where(mask, -inf, x)도 돌리지 못한다. 둘 다 같은 결과의 단순한 연산으로 바꾼다.
consts = {i.name: numpy_helper.to_array(i) for i in g.initializer}
consts.update({n.output[0]: numpy_helper.to_array(n.attribute[0].t) for n in g.node if n.op_type == "Constant"})
byname = {n.name: n for n in g.node}
gnd = byname["model/p_node_GatherND_7"]
assert list(consts[gnd.input[1]].reshape(-1)) == list(range(-9, 0))
t_in, t_out = byname["model/p_node_Transpose_0"], byname["model/p_node_index"]
g.initializer.extend([numpy_helper.from_array(np.array([-9], np.int64), "last9_start"),
                      numpy_helper.from_array(np.array([np.iinfo(np.int64).max], np.int64), "last9_end"),
                      numpy_helper.from_array(np.array([1], np.int64), "last9_axis")])
sl = helper.make_node("Slice", [t_in.input[0], "last9_start", "last9_end", "last9_axis"], [t_out.output[0]],
                      name="last9_slice")
wh = byname["model/p_node_masked_fill"]
mask = consts[wh.input[0]]
# 녹화 주행에서 잰 score는 [-6.3, 4.7]이다: -30이면 가린 가중치가 e^-19 미만으로 남고
# 양자화 범위도 좁게 유지된다(-inf/-1e4로 채우면 범위가 망가진다)
g.initializer.append(numpy_helper.from_array(np.where(mask, -30.0, 0.0).astype(np.float32), "attn_mask_add"))
add = helper.make_node("Add", [wh.input[2], "attn_mask_add"], [wh.output[0]], name="attn_mask_add")
new = []
for n in g.node:
    if n is gnd: new.append(sl)
    elif n is wh: new.append(add)
    elif n is t_in or n is t_out: continue
    else: new.append(n)
del g.node[:]
g.node.extend(new)

# Pulsar2는 x / expand(clip(ReduceL2(x)))를 LpNormalization으로 합치는데, AX620E
# 백엔드는 이를 3D/4D에서만 타일링한다. 두 [1,512] L2 정규화를 [1,1,512] 뷰에서
# 돌린다.
prod = {o: n for n in g.node for o in n.output}
g.initializer.extend([numpy_helper.from_array(np.array([1, 1, 512], np.int64), "l2n_3d"),
                      numpy_helper.from_array(np.array([1, 512], np.int64), "l2n_2d")])
new = []
for n in g.node:
    e = prod.get(n.input[1]) if n.op_type == "Div" and len(n.input) == 2 else None
    c = prod.get(e.input[0]) if e is not None and e.op_type == "Expand" else None
    r = prod.get(c.input[0]) if c is not None and c.op_type == "Clip" else None
    if r is not None and r.op_type == "ReduceL2" and r.input[0] == n.input[0]:
        x, y = n.input[0], n.output[0]
        new.append(helper.make_node("Reshape", [x, "l2n_3d"], [x + "_3d"], name=n.name + "_to3d"))
        r.input[0] = x + "_3d"; e.input[1] = "l2n_3d"
        n.input[0] = x + "_3d"; n.output[0] = y + "_3d"
        new.append(n)
        new.append(helper.make_node("Reshape", [y + "_3d", "l2n_2d"], [y], name=n.name + "_to2d"))
    else:
        new.append(n)
# ReduceL2 axes는 3D 뷰의 마지막 차원을 가리켜야 한다
for n in new:
    if n.op_type == "ReduceL2":
        ax = n.input[1] if len(n.input) > 1 else None
        if ax:
            g.initializer.append(numpy_helper.from_array(np.array([-1], np.int64), n.name + "_ax"))
            n.input[1] = n.name + "_ax"
del g.node[:]
g.node.extend(new)
# 위상 정렬: 새 Reshape가 자기 ReduceL2보다 앞서야 한다
order, seen, avail = [], set(), {i.name for i in g.input} | {i.name for i in g.initializer} | {""}
pending = list(g.node)
while pending:
    rest = []
    for n in pending:
        if all(i in avail for i in n.input): order.append(n); avail.update(n.output)
        else: rest.append(n)
    assert len(rest) < len(pending), "cycle"
    pending = rest
del g.node[:]
g.node.extend(order)
del g.value_info[:]
m = onnx.shape_inference.infer_shapes(m)
onnx.checker.check_model(m)
onnx.save(m, dst)
print([(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim]) for i in m.graph.input])
