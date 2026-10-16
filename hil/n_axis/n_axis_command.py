#!/usr/bin/env python3
"""One-shot N-axis request, monotonic keepalive and status monitor. Never restarts."""
import argparse
import math
import secrets
import time

class Session:
    def __init__(self, axes, kp=20., kd=1.):
        if axes not in (2,3) or not math.isfinite(kp) or not 0<kp<=20 or not math.isfinite(kd) or not 0<=kd<=1:
            raise ValueError('N=2/3, 0<Kp<=20, 0<=Kd<=1 required')
        self.axes,self.kp,self.kd=axes,kp,kd
        self.nonce=secrets.randbits(64) or 1; self.id=1; self.sequence=0; self.started=False
    def request(self, state, now):
        if self.started or state!=1: raise ValueError('start once, and only from IDLE')
        self.started=True
        return dict(nonce=self.nonce,id=self.id,sent_ns=now,axes=self.axes,amplitude=math.pi/6,
                    frequency=.2,cycles=5.,ramp_cycles=1.,hold=1.,kp=[self.kp]*3,kd=[self.kd]*3,release=True)
    def heartbeat(self, now):
        if not self.started: raise ValueError('not started')
        self.sequence+=1
        return dict(nonce=self.nonce,id=self.id,sequence=self.sequence,sent_ns=now)

def outcome(state, terminal, record_state):
    if state==7 or record_state==2 or (terminal and state not in (5,6)): return 1
    if state in (5,6): return 0
    return None

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['start','stop','finish']);p.add_argument('--controller',default='/n_axis')
    p.add_argument('--axes',type=int,choices=[2,3]);p.add_argument('--kp',type=float,default=20.);p.add_argument('--kd',type=float,default=1.)
    a=p.parse_args(argv)
    if a.action=='start' and a.axes is None:p.error('--axes is required for start')
    try:
        session=Session(a.axes,a.kp,a.kd) if a.action=='start' else None
    except ValueError as e:
        p.error(str(e))
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from bxi_hardware.msg import NAxisKeepalive,NAxisStatus
    from bxi_hardware.srv import StartNAxis
    from std_srvs.srv import Trigger
    rclpy.init(); node=rclpy.create_node('n_axis_command')
    prefix=a.controller.rstrip('/')+'/n_axis/'; qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
    status=[None,0.]
    def receive(m):status[:]=[m,time.monotonic()]
    node.create_subscription(NAxisStatus,prefix+'status',receive,qos)
    pub=node.create_publisher(NAxisKeepalive,prefix+'keepalive',qos)
    clients={x:node.create_client(StartNAxis if x=='start' else Trigger,prefix+x) for x in ('start','finish','stop')}
    interrupted=False; stop_sent=False
    try:
        if not clients[a.action].wait_for_service(timeout_sec=5.):raise RuntimeError('service unavailable')
        deadline=time.monotonic()+5
        while status[0] is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if status[0] is None:raise RuntimeError('status unavailable')
        if a.action=='finish' and status[0].state!=4:raise RuntimeError('finish requires HOLDING')
        if session:
            request=StartNAxis.Request(**session.request(status[0].state,time.monotonic_ns()))
        else:request=Trigger.Request()
        stop_sent = a.action == 'stop'
        future=clients[a.action].call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        if not future.done():raise RuntimeError('service timed out; request will not be repeated')
        response=future.result()
        if not (response.accepted if session else response.success):raise RuntimeError(response.message)
        if not session:
            m=status[0]
            print(f'{response.message}; observed_state={m.state} record_state={m.record_state} csv={m.csv_path or "not saved"}');return 0
        next_heart=time.monotonic()
        while rclpy.ok():
            m=status[0]; result=outcome(m.state,m.terminal,m.record_state)
            if result is not None:
                interrupted = result != 0
                label={5:'RELEASED',6:'STOPPED',7:'ABORTED'}.get(m.state,str(m.state))
                print(f'outcome={"ABNORMAL" if result else "NORMAL"} state={label} terminal={m.terminal} record_state={m.record_state} csv={m.csv_path or "not saved"}')
                return result
            if time.monotonic()-status[1]>1:raise RuntimeError('status lost; no automatic restart')
            if time.monotonic()>=next_heart:
                pub.publish(NAxisKeepalive(**session.heartbeat(time.monotonic_ns())))
                next_heart=time.monotonic()+.05
            rclpy.spin_once(node,timeout_sec=.01)
    except (Exception,KeyboardInterrupt) as e:
        interrupted=True;print(f'ERROR: {e}; keepalive stopped; no restart')
        return 1
    finally:
        if interrupted and not stop_sent and rclpy.ok():
            # Exactly one best-effort stop, no retry and no enable/power operation.
            if clients['stop'].service_is_ready():
                future=clients['stop'].call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node,future,timeout_sec=.2)
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
