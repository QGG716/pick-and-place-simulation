"""Restore an explicitly saved NetworkManager profile before robot TCP IO.

Called by the robot device worker, never by constructors or Tk. This activates
an existing profile only; it cannot create profiles, guess addresses, change the
default route, or issue robot/valve commands.
"""
import ipaddress
import json
import os
import subprocess
from ..controlled.ui import ManagedRobotView


class SavedRobotNetwork:
    def __init__(self, config, *, run=subprocess.run):
        self.config=config;self.run=run;self.status='等待加载已保存网络配置'

    def command(self, args, timeout=4):
        try:
            result=self.run(args,capture_output=True,text=True,timeout=timeout,
                            env=dict(os.environ,LC_ALL='C'))
        except FileNotFoundError as exc:
            raise RuntimeError('ROBOT_NETWORK_TOOL_MISSING: 系统缺少 nmcli 或 ip 命令') from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError('ROBOT_NETWORK_TIMEOUT: 恢复已保存网络配置超时，请检查网线和系统网络权限') from exc
        if result.returncode:
            detail=(result.stderr or result.stdout).strip()
            raise RuntimeError('ROBOT_NETWORK_FAILED: '+detail)
        return result.stdout.strip()

    def ensure(self, host):
        c=self.config
        try:
            if c.get('manager')!='NetworkManager' or not c.get('connection'):
                raise ValueError('ROBOT_NETWORK_CONFIG_MISSING: 请指定已保存的 NetworkManager 连接名称或 UUID')
            ipaddress.IPv4Address(host)
            self.status='正在读取网络配置 '+c['connection']
            fields='connection.id,connection.uuid,connection.type,connection.interface-name,ipv4.never-default'
            values=self.command(['nmcli','--get-values',fields,'connection','show',c['connection']]).splitlines()
            if len(values)!=5:raise ValueError('ROBOT_NETWORK_PROFILE_INVALID: 无法读取已保存配置')
            name,uuid,kind,bound,never_default=values
            interface=c.get('interface') or bound
            if kind!='802-3-ethernet' or not interface or interface=='--' or (bound not in ('','--',interface)):
                raise ValueError('ROBOT_NETWORK_INTERFACE_MISMATCH: 已保存的有线网络配置与指定网卡不一致')
            if never_default!='yes':
                raise ValueError('ROBOT_NETWORK_DEFAULT_ROUTE: 机器人专用网络须保存为 ipv4.never-default=yes')
            active=self.command(['nmcli','--get-values','GENERAL.CON-UUID','device','show',interface])
            if active not in ('','--',uuid):
                raise ValueError('ROBOT_NETWORK_INTERFACE_IN_USE: '+interface+' 正在使用其他网络配置，请在系统网络设置中处理')
            activated=active!=uuid
            if activated:
                self.status='正在启用已保存网络 '+name+' / '+interface
                self.command(['nmcli','--wait','8','connection','up','uuid',uuid,'ifname',interface],timeout=10)
            route=json.loads(self.command(['ip','-j','-4','route','get',host]))[0]
            if route.get('dev')!=interface or not route.get('prefsrc'):
                raise ValueError('ROBOT_NETWORK_ROUTE_MISMATCH: 控制器 '+host+' 未经 '+interface+' 的有效本机地址连接')
            self.status=f"网络已就绪 · {name} · {interface} · {route['prefsrc']} → {host}"
            return dict(connection=name,uuid=uuid,interface=interface,source_ip=route['prefsrc'],
                        robot_host=host,activated_saved_profile=activated,status='READY')
        except Exception as exc:
            self.status='网络配置失败 · '+str(exc)
            raise


class NetworkedRobotView(ManagedRobotView):
    """The single-carton facade restores host networking before the SDK owner."""
    def __init__(self, service, mapping, config, *, network=None):
        super().__init__(service,mapping)
        self.network=network if network is not None else SavedRobotNetwork(config)

    def connect(self, host, port):
        if self.connected:return super().connect(host,port)
        ready=self.network.ensure(host)
        result=super().connect(host,port)
        self.capabilities=dict(result,host_network=ready)
        return self.capabilities
