"""Site assessment and operator chat prompts for the Qwen VLM, and strict parsing.

The assessment asks two questions about the live colour frame: may
pick-and-place start, and does the carton stack show a collapse risk. The
reply must be one JSON object with fixed enumerations; anything else is
INVALID_OUTPUT and is shown as such, never read as "safe". Every result
carries authority ADVISORY_ONLY_NO_GATE (user decision 2026-11-28): it is
displayed and spoken, it does not enter a plan, an execute gate or a job.
"""
from dataclasses import dataclass, field, asdict
import hashlib
import json
import re

from . import AUTHORITY

CAN_START = ('YES', 'NO', 'UNCERTAIN')
RISK = ('LOW', 'MEDIUM', 'HIGH', 'UNCERTAIN')
CAN_START_ZH = dict(YES='可以开始', NO='不建议开始', UNCERTAIN='无法判断')
RISK_ZH = dict(LOW='低', MEDIUM='中', HIGH='高', UNCERTAIN='无法判断')

ASSESS_SYSTEM = (
    '你是 ECO65 六轴机械臂纸箱拆垛工作站的视觉安全顾问。图像来自装在机械臂 1 轴上的 D435i 相机，'
    '正对待抓取的纸箱堆。机械臂用真空吸盘从箱堆正面一个一个吸取纸箱，搬到另一侧释放。\n'
    '只根据图像中实际看得到的内容判断，看不清或看不到的部分要如实说明，不要猜测。\n'
    '判断两件事：\n'
    '1. 现在是否适合开始抓放：工作区内有没有人或人的肢体、有没有掉落/散落的纸箱或异物挡在机械臂和箱堆之间、'
    '箱堆是否在视野内且清楚可见、光线是否足以看清。\n'
    '2. 箱堆是否有坍塌风险：箱子是否明显倾斜、错位悬空（上箱超出下箱边缘过多）、被压变形或破损、'
    '整体歪斜、上重下轻、箱子之间有大缝隙，或者抽走最前面/最上面的箱子后其它箱子可能失去支撑。\n'
    '只输出一个 JSON 对象，不要输出其它文字，格式：\n'
    '{"can_start": "YES|NO|UNCERTAIN", "collapse_risk": "LOW|MEDIUM|HIGH|UNCERTAIN", '
    '"people_in_area": true|false|null, "hazards": ["具体隐患，中文，每条一句"], '
    '"reasons": ["判断依据，中文，每条一句"], "summary": "一句话中文结论，不超过40字"}')

CHAT_SYSTEM = (
    '你是 ECO65 纸箱拆垛工作站的现场语音助手，和操作员实时对话。你能看到相机当前画面和系统状态（JSON）。\n'
    '回答要简短口语化，适合朗读，一般不超过 60 个字；不确定就直说不确定。\n'
    '你只能提供信息和建议：你不能启动作业、移动机械臂、开关真空或改任何参数，这些只能由操作员在界面上操作。'
    '如果操作员要求停下机械臂，告诉操作员直接说“停止”两个字或按界面上的结束作业/急停。\n'
    '你的判断只作提示，不影响系统的任何安全检查。')

MAX_ITEMS = 6
MAX_TEXT = 120


@dataclass(frozen=True)
class SiteAssessment:
    status: str                     # OK | INVALID_OUTPUT | ERROR
    can_start: str = 'UNCERTAIN'
    collapse_risk: str = 'UNCERTAIN'
    people_in_area: object = None
    hazards: tuple = ()
    reasons: tuple = ()
    summary: str = ''
    raw_text: str = ''
    error: str = ''
    image_sha256: str = ''
    rgb_number: object = None
    stamp: float = 0.
    timing: dict = field(default_factory=dict)
    authority: str = AUTHORITY

    @property
    def level(self):
        """ALERT / CAUTION / OK for display colour and speech; never a gate."""
        if self.status != 'OK':
            return 'CAUTION'
        if self.can_start == 'NO' or self.collapse_risk == 'HIGH' or self.people_in_area is True:
            return 'ALERT'
        if self.can_start == 'UNCERTAIN' or self.collapse_risk in ('MEDIUM', 'UNCERTAIN'):
            return 'CAUTION'
        return 'OK'

    def headline(self):
        if self.status != 'OK':
            return f'VLM 评估无效（{self.status}）：' + (self.error or self.raw_text[:80])
        people = {True: '有人', False: '无人', None: '人员不确定'}[self.people_in_area]
        return f'{CAN_START_ZH[self.can_start]} · 坍塌风险{RISK_ZH[self.collapse_risk]} · {people}'

    def spoken(self):
        if self.status != 'OK':
            return '视觉评估失败，请人工确认现场。'
        text = self.summary or self.headline()
        if self.collapse_risk == 'HIGH':
            text = '注意，箱堆有坍塌风险。' + text
        elif self.people_in_area is True:
            text = '注意，工作区内可能有人。' + text
        return text

    def record(self):
        d = asdict(self)
        d['level'] = self.level
        return d


def _texts(value):
    if not isinstance(value, list):
        return ()
    return tuple(str(v).strip()[:MAX_TEXT] for v in value[:MAX_ITEMS] if isinstance(v, (str, int, float)) and str(v).strip())


def extract_json(text):
    """The first JSON object in the reply (models sometimes wrap it in ```json)."""
    decoder = json.JSONDecoder()
    start = text.find('{')
    while start >= 0:
        try:
            value, _ = decoder.raw_decode(text, start)
        except ValueError:
            value = None
        if isinstance(value, dict):
            return value
        start = text.find('{', start + 1)
    return None


def parse_assessment(text, **meta):
    value = extract_json(text or '')
    if value is None:
        return SiteAssessment('INVALID_OUTPUT', raw_text=text or '', error='回复中没有 JSON 对象', **meta)
    can_start = str(value.get('can_start', '')).strip().upper()
    risk = str(value.get('collapse_risk', '')).strip().upper()
    people = value.get('people_in_area')
    problems = []
    if can_start not in CAN_START:
        problems.append(f'can_start={can_start!r}')
    if risk not in RISK:
        problems.append(f'collapse_risk={risk!r}')
    if people not in (True, False, None):
        problems.append(f'people_in_area={people!r}')
    if problems:
        return SiteAssessment('INVALID_OUTPUT', raw_text=text, error='字段不合规：' + '，'.join(problems), **meta)
    summary = value.get('summary')
    return SiteAssessment('OK', can_start=can_start, collapse_risk=risk, people_in_area=people,
                          hazards=_texts(value.get('hazards')), reasons=_texts(value.get('reasons')),
                          summary=str(summary).strip()[:MAX_TEXT] if isinstance(summary, str) else '', raw_text=text, **meta)


def context_text(context):
    return json.dumps(context or {}, ensure_ascii=False, sort_keys=True, default=str)[:3000]


def assessment_messages(context):
    return [dict(role='system', content=[dict(type='text', text=ASSESS_SYSTEM)]),
            dict(role='user', content=[dict(type='image'),
                                       dict(type='text', text='系统状态（仅供参考，以图像为准）：' + context_text(context)
                                            + '\n请按要求只输出 JSON。')])]


def chat_messages(history, question, context, with_image):
    """System + earlier turns (text only, bounded) + this turn with the current frame."""
    messages = [dict(role='system', content=[dict(type='text', text=CHAT_SYSTEM)])]
    for role, text in history[-8:]:
        messages.append(dict(role=role, content=[dict(type='text', text=text)]))
    content = [dict(type='image')] if with_image else []
    content.append(dict(type='text', text='当前系统状态：' + context_text(context) + '\n操作员：' + question))
    messages.append(dict(role='user', content=content))
    return messages


def prompt_sha256(messages):
    return hashlib.sha256(json.dumps(messages, ensure_ascii=False, sort_keys=True).encode()).hexdigest()


def clean_reply(text):
    """Spoken reply: no markdown, single paragraph."""
    text = re.sub(r'[`*#>]+', '', text or '')
    return re.sub(r'\s+', ' ', text).strip()
