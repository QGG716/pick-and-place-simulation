"""Display actual task states without declaring success from sent point counts."""
import pytest
from unloading_sim.workbench.controlled.ui import execution_message


@pytest.mark.parametrize('status,expected',[
    ({'state':'READY','task_id':None},None),
    ({'state':'PREPARING'},'任务准备中'),
    ({'state':'EXECUTING','sent_points':3,'total_points':10},'已发送 3/10'),
    ({'state':'EXECUTING','sent_points':10,'total_points':10},'尚未报告成功'),
    ({'state':'SUCCEEDED','arrival_confirmed':False},'缺少实测到位确认'),
    ({'state':'SUCCEEDED','arrival_confirmed':True},'执行完成'),
    ({'state':'STOPPING','reason':'SDK_SEND_ERROR'},'等待真实反馈确认'),
    ({'state':'CANCELLED','stop_confirmed':True},'已通过真实反馈确认停止'),
    ({'state':'FAILED','reason':'SDK_SEND_ERROR','stop_confirmed':True},'SDK_SEND_ERROR；已确认停止'),
    ({'state':'STOP_UNCONFIRMED','reason':'FEEDBACK_EXPIRED'},'停止未确认'),
])
def test_execution_state_messages(status,expected):
    result=execution_message(dict({'task_id':'test'},**status))
    if expected is None:assert result is None
    else:assert expected in result
