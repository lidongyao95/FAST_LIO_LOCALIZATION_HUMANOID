#!/usr/bin/env python3
"""Synthetic point-cloud and threshold transaction tests (NumPy/SciPy, no ROS)."""
from concurrent.futures import Future
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'navigation'))
from obstacle_cloud import consensus_mask, filter_cloud, records_and_xyz
from planning_obstacles import DEFAULT_THRESHOLDS, ObstacleGate, validate_thresholds
from planning_inflation import PlanningInflation


def cloud(points, *, big=False, double=False, rows=1, padding=0):
    size = 8 if double else 4
    dtype = np.dtype(dict(names=['x', 'y', 'z', 'extra'], formats=[('>' if big else '<') + 'f' + str(size)]*3+['u4'],
                          offsets=[0,size,size*2,size*3], itemsize=size*3+4))
    records = np.zeros(len(points), dtype=dtype)
    for i, name in enumerate(('x', 'y', 'z')):
        records[name] = np.array(points)[:,i]
    records['extra'] = np.arange(len(points)) + 100
    width = len(points)//rows
    data = b''.join(records[i*width:(i+1)*width].tobytes()+b'\x7f'*padding for i in range(rows))
    return NS(width=width, height=rows, point_step=dtype.itemsize,
        row_step=width*dtype.itemsize+padding, is_bigendian=big, is_dense=False, data=data,
        header=NS(frame_id='body', stamp=17),
        fields=[NS(name=name, offset=i*size, count=1, datatype=8 if double else 7) for i,name in enumerate(('x','y','z'))])


class CloudTests(unittest.TestCase):
    def filtered(self, points, radius=.1, minimum=3, **layout):
        message = cloud(points, **layout)
        return message, filter_cloud(message, dict(neighbor_radius=radius,min_points=minimum),
                                     ([0,0,0],[0,0,0,1]), .15, 1.8, 8.)

    def test_consensus_rejects_single_noise_and_radius_changes_result(self):
        points = [[1,0,.5],[1.04,0,.5],[1.08,0,.5],[2,0,.5]]
        _, (marked,rays,stats) = self.filtered(points)
        self.assertEqual(marked.width,3)
        self.assertEqual(rays.width,4)
        self.assertEqual(stats,dict(input_points=4,candidate_points=4,obstacle_points=3))
        _, (marked,_,_) = self.filtered(points,radius=.03)
        self.assertEqual(marked.width,0)

    def test_minimum_counts_self_and_threshold_one_preserves_candidates(self):
        points=[[1,0,.5],[2,0,.5]]
        _, (marked,_,_) = self.filtered(points,minimum=1)
        self.assertEqual(marked.width,2)
        _, (marked,_,_) = self.filtered(points,minimum=2)
        self.assertEqual(marked.width,0)

    def test_ground_and_out_of_range_points_cannot_vote_for_obstacles(self):
        points=[[1,0,.16],[1,0,.14],[1,0,.13],[7.99,0,.2],[8.01,0,.2],[8.02,0,.2]]
        _, (marked,rays,stats) = self.filtered(points)
        self.assertEqual(marked.width,0)
        self.assertEqual(stats['candidate_points'],2)
        self.assertEqual(rays.width,6)

    def test_height_uses_map_transform_before_consensus(self):
        message=cloud([[1,0,-.8],[1.01,0,-.8]])
        marked,_,_=filter_cloud(message,dict(neighbor_radius=.1,min_points=2),
                               ([0,0,1.3],[0,0,0,1]),.15,1.8,8.)
        self.assertEqual(marked.width,2)
        self.assertLess(records_and_xyz(marked)[1][0,2],0)

    def test_big_endian_padded_rows_preserve_complete_original_records(self):
        for double in (False,True):
            original,(marked,rays,_) = self.filtered([[1,0,.5],[1.02,0,.5],[2,0,.5],[3,0,.5]],
                minimum=2,big=True,double=double,rows=2,padding=11)
            records,_=records_and_xyz(original)
            self.assertEqual(marked.data,records[:2].tobytes())
            self.assertEqual(rays.data,records.tobytes())
            self.assertEqual(marked.header,original.header)
            self.assertEqual((original.width,original.height),(2,2))

    def test_nonfinite_values_are_removed_from_marking_and_clearing(self):
        _, (marked,rays,stats) = self.filtered([[1,0,.5],[float('nan'),0,.5],[float('inf'),0,.5]],minimum=1)
        self.assertEqual((marked.width,rays.width),(1,1))
        self.assertEqual(stats['input_points'],3)

    def test_consensus_is_three_dimensional_and_not_cluster_connectivity(self):
        points=np.array([[0,0,0],[0,0,.09],[0,0,.18],[0,0,.27]])
        self.assertEqual(consensus_mask(points,.1,3).tolist(),[False,True,True,False])
        self.assertEqual(consensus_mask(points,.1,4).sum(),0)

    def test_kd_result_matches_brute_force_distances(self):
        points=np.random.default_rng(19).uniform(0,1,(150,3))
        counts=(np.linalg.norm(points[:,None,:]-points[None,:,:],axis=2)<=.25).sum(axis=1)
        np.testing.assert_array_equal(consensus_mask(points,.25,8),counts>=8)

    def test_ros_publisher_uses_cloud_stamp_and_separates_marks_from_rays(self):
        from planning_obstacles import RosPlanningObstacles
        clock,tf_module=ModuleType('rclpy.time'),ModuleType('tf2_ros')
        clock.Time=NS(from_msg=lambda stamp:('exact_stamp',stamp))
        tf_module.TransformException=RuntimeError
        published,rays,calls=[],[],[]
        gate=RosPlanningObstacles.__new__(RosPlanningObstacles)
        gate.thresholds=dict(DEFAULT_THRESHOLDS,min_points=3,neighbor_radius=.1)
        gate.geometry=('map',.15,1.8,8.)
        gate.publisher=NS(publish=published.append)
        gate.clearing_publisher=NS(publish=rays.append)
        def lookup(*args):
            calls.append(args)
            return NS(transform=NS(translation=NS(x=0,y=0,z=0),rotation=NS(x=0,y=0,z=0,w=1)))
        gate.tf_buffer=NS(lookup_transform=lookup)
        original=cloud([[1,0,.5],[1.04,0,.5],[1.08,0,.5],[2,0,.5]])
        before=original.data
        with patch.dict(sys.modules,{'rclpy.time':clock,'tf2_ros':tf_module}):
            gate._publish(original)
        self.assertEqual((published[0].width,rays[0].width),(3,4))
        self.assertEqual(calls[0],('map','body',('exact_stamp',17)))
        self.assertEqual(original.data,before)

    def test_truncated_or_invalid_layout_is_rejected(self):
        message=cloud([[1,0,.5]])
        message.data=message.data[:-1]
        with self.assertRaisesRegex(ValueError,'不完整'):records_and_xyz(message)
        message=cloud([[1,0,.5]])
        message.fields[0].count=2
        with self.assertRaises(ValueError):records_and_xyz(message)


class TransactionTests(unittest.TestCase):
    def setUp(self):
        self.events=[]
        self.fail_clear=False
        def clear():
            self.events.append(('clear',self.gate.flushing))
            if self.fail_clear:
                self.fail_clear=False
                raise RuntimeError('clear failed')
        self.gate=ObstacleGate(lambda:None,clear,lambda p:self.events.append(('cloud',self.gate.flushing)),
            lambda p:None,updated=lambda:self.events.append(('updated',True)),
            set_inflation=lambda radius:self.events.append(('inflate',radius)),timeout=.2)
        self.gate.receive(1)
        self.gate.refresh()
        self.stop=threading.Event()
        self.worker=threading.Thread(target=self.produce)
        self.worker.start()
        self.addCleanup(self.finish)

    def produce(self):
        while not self.stop.wait(.005):self.gate.receive(1)

    def finish(self):
        self.stop.set()
        self.worker.join(1)

    def test_threshold_transaction_flushes_cache_and_waits_for_new_costmap(self):
        values=dict(neighbor_radius=.2,min_points=4,inflation_radius=.8)
        state=self.gate.configure(values)
        self.assertEqual(state['thresholds'],values)
        self.assertFalse(state['busy'])
        self.assertTrue(state['available'])
        self.assertEqual([e for e in self.events if e[0]=='clear'],[('clear',True)])
        self.assertEqual(self.events[-1],('updated',True))
        self.assertEqual(state['revision'],1)

    def test_clear_failure_restores_all_settings_and_rebuilds_old_observations(self):
        self.fail_clear=True
        with self.assertRaisesRegex(RuntimeError,'clear failed'):
            self.gate.configure(dict(neighbor_radius=.2,min_points=4,inflation_radius=.8))
        self.assertEqual(self.gate.thresholds,DEFAULT_THRESHOLDS)
        self.assertEqual([e for e in self.events if e[0]=='inflate'],[('inflate',.8),('inflate',.6)])
        self.assertEqual(len([e for e in self.events if e[0]=='clear']),2)
        self.assertEqual(self.events[-1],('updated',True))
        self.assertIsNone(self.gate.state()['enabled'])
        self.assertEqual(self.gate.revision,2)

    def test_unconfirmed_rollback_cannot_reappear_as_healthy_on_poll(self):
        self.gate.clear=lambda:(_ for _ in ()).throw(RuntimeError('clear offline'))
        with self.assertRaises(RuntimeError):
            self.gate.configure(dict(DEFAULT_THRESHOLDS,min_points=4))
        self.gate.refresh()
        self.assertFalse(self.gate.state()['available'])
        self.assertIn('重启 Nav2',self.gate.state()['message'])

    def test_concurrent_settings_are_serialized(self):
        self.gate.busy=True
        with self.assertRaises(RuntimeError):
            self.gate.configure(dict(DEFAULT_THRESHOLDS,min_points=4))
        with self.assertRaises(RuntimeError):self.gate.set_enabled(False)
        self.gate.busy=False

    def test_filter_change_does_not_reload_inflation(self):
        self.gate.configure(dict(DEFAULT_THRESHOLDS,min_points=4))
        self.assertFalse(any(e[0]=='inflate' for e in self.events))

    def test_noop_does_not_invalidate_paths(self):
        self.assertFalse(self.gate.configure(DEFAULT_THRESHOLDS)['busy'])
        self.assertEqual(self.gate.revision,0)

    def test_invalid_settings_never_mutate_or_clear(self):
        for value in (dict(DEFAULT_THRESHOLDS,min_points=True),dict(DEFAULT_THRESHOLDS,min_points=2.5),
            dict(DEFAULT_THRESHOLDS,neighbor_radius=float('nan')),dict(DEFAULT_THRESHOLDS,inflation_radius=2),
            dict(DEFAULT_THRESHOLDS,min_points=0),{'min_points':4},dict(DEFAULT_THRESHOLDS,extra=1)):
            with self.assertRaises(ValueError):self.gate.configure(value)
        self.assertEqual(self.gate.thresholds,DEFAULT_THRESHOLDS)
        self.assertFalse(any(e[0]=='clear' for e in self.events))


class InflationTests(unittest.TestCase):
    def setUp(self):
        lifecycle=ModuleType('lifecycle_msgs.srv')
        lifecycle.GetState=NS(Request=NS)
        lifecycle.ChangeState=NS(Request=lambda:NS(transition=NS(id=0)))
        interfaces=ModuleType('rcl_interfaces.srv')
        interfaces.SetParametersAtomically=NS(Request=NS)
        messages=ModuleType('rcl_interfaces.msg')
        messages.Parameter=messages.ParameterValue=NS
        messages.ParameterType=NS(PARAMETER_DOUBLE=3)
        self.patcher=patch.dict(sys.modules,{'lifecycle_msgs.srv':lifecycle,
            'rcl_interfaces.srv':interfaces,'rcl_interfaces.msg':messages})
        self.patcher.start();self.addCleanup(self.patcher.stop)
        self.state=3;self.calls=[];self.radius=.6;self.reject=False
        def client(service,name,**kw):
            def call(request):
                future=Future()
                if name.endswith('get_state'):
                    result=NS(current_state=NS(id=self.state))
                elif name.endswith('change_state'):
                    transition=request.transition.id
                    self.calls.append(transition)
                    self.state={4:2,2:1,1:2,3:3}[transition]
                    result=NS(success=True)
                else:
                    if not self.reject:self.radius=request.parameters[0].value.double_value
                    result=NS(result=NS(successful=not self.reject,reason='rejected'))
                future.set_result(result);return future
            return NS(service_is_ready=lambda:True,call_async=call)
        from planning_obstacles import wait_all
        self.controller=PlanningInflation(NS(create_client=client),object(),wait_all)

    def test_foxy_reconfigures_so_radius_reaches_configure_only_plugin(self):
        with patch.dict('os.environ',{'ROS_DISTRO':'foxy'}):self.controller.apply(.8)
        self.assertEqual(self.calls,[4,2,1,3]);self.assertEqual(self.radius,.8)

    def test_humble_uses_runtime_callback_without_reconfiguring(self):
        with patch.dict('os.environ',{'ROS_DISTRO':'humble'}):self.controller.apply(.8)
        self.assertEqual(self.calls,[]);self.assertEqual(self.radius,.8)

    def test_failed_parameter_write_does_not_reload_or_change_radius(self):
        self.reject=True
        with patch.dict('os.environ',{'ROS_DISTRO':'foxy'}),self.assertRaises(RuntimeError):self.controller.apply(.8)
        self.assertEqual(self.calls,[]);self.assertEqual(self.radius,.6)

    def test_rollback_can_resume_partially_reloaded_foxy(self):
        self.state=1
        with patch.dict('os.environ',{'ROS_DISTRO':'foxy'}):self.controller.apply(.6)
        self.assertEqual(self.calls,[1,3]);self.assertEqual(self.state,3)


if __name__=='__main__':unittest.main()
