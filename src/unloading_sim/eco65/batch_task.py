"""One cold, continuous native task unloading the entire frozen inventory."""
from copy import deepcopy
import gzip
import json
from time import perf_counter

from ..moveit2_backend import digest
from .model import save_json, check_transform
from .native_contact import face_outward
from .coupled_task import CoupledTask
from .coupled_state import validate_cycle

ALL_CARTONS = 'ALL_CARTONS_DRAINED'


class BatchTask(CoupledTask):
    completion_goal = ALL_CARTONS

    def __init__(self, *args, carton_seconds=900., **kwargs):
        if not 30. <= carton_seconds <= 1800.:
            raise ValueError('BATCH_CARTON_BUDGET_OUT_OF_RANGE')
        self.carton_seconds = float(carton_seconds)
        self.process_history = []
        self.carton_budgets = []
        super().__init__(*args, **kwargs)
        self.batch_deadline = self.deadline
        self.begin_carton_budget()

    def begin_carton_budget(self):
        start = perf_counter()
        self.deadline = min(self.batch_deadline, start+self.carton_seconds)
        self.reception_deadline = None
        self.carton_budgets.append(dict(carton_number=len(self.completed_ids)+1,
            start_elapsed_s=start-self.started, deadline_elapsed_s=self.deadline-self.started))

    def request(self, *args, **kwargs):
        return dict(super().request(*args, **kwargs), completion_goal=ALL_CARTONS)

    def call(self, request):
        if request['op'] != 'init':
            request = dict(request, completion_goal=ALL_CARTONS)
        return super().call(request)

    def checkpoint(self):
        return dict(super().checkpoint(), process_history=deepcopy(self.process_history))

    def restore(self, cp):
        super().restore(cp)
        self.process_history = deepcopy(cp['process_history'])

    def finish_release(self, *args, **kwargs):
        super().finish_release(*args, **kwargs)
        self.process_history.append(deepcopy(self.adopted_process))

    def approach(self, grasp):
        """A valid pregrasp hands its actual q to a separately bounded contact.

        The pregrasp search has one absolute cutoff across offsets/seeds. Contact
        gets its own finite motion budget; the carton/batch clock never resets.
        """
        self.select_grasp(grasp)
        contact=check_transform(grasp['tcp_pose_D'])
        prior=self.connection_deadline
        with self.connection():
            approach_deadline=self.connection_deadline
            seeds=[q for q in self.ik(contact,self.current_q,'contact')
                   if self.direction_goal_valid(self.current_q,q,'contact')]
            seeds.sort(key=lambda q:self.goal_cost(self.current_q,q))
        if not seeds:raise ValueError('NATIVE_CONTACT_IK_EXHAUSTED')
        for seed,distance in [(q,d) for q in seeds[:3] for d in (.025,.01,.005)]:
            if perf_counter()>=approach_deadline:
                raise ValueError('NATIVE_APPROACH_SEARCH_BUDGET_EXHAUSTED')
            cp=self.checkpoint()
            pre=contact.copy();pre[:3,3]+=face_outward(self.target_pose,self.face)*distance
            try:
                self.connection_deadline=approach_deadline
                self.move('pregrasp',pose=pre,seed=seed)
                self.connection_deadline=prior
                self.move('contact',pose=contact)
                if self.seal()['status']!='PASS':raise ValueError('ACTUAL_CONTACT_SEAL_FAILED')
                return
            except ValueError as error:
                self.cycle_failures.append(dict(stage='approach',target_id=self.target['id'],
                    candidate_id=grasp['candidate_id'],offset_m=distance,contact_ik_q=seed.tolist(),error=str(error)))
                self.restore(cp)
            finally:
                self.connection_deadline=prior
        raise ValueError('NATIVE_APPROACH_CONTACT_EXHAUSTED')

    def release_candidates(self, height, yaws=(0.,)):
        # A top grasp preserves the upright carton orientation during withdrawal.
        # Try that existing orientation before extra loaded wrist rotations.
        if self.face=='top':yaws=sorted(yaws,key=abs)
        yield from super().release_candidates(height,yaws)

    def return_next(self):
        # A completed carton is a committed batch prefix. Failure later in the
        # batch must not silently restart the initial carton at a different q.
        try:
            while len(self.completed_ids) < len(self.original_boxes):
                self.begin_carton_budget()
                drained=self.checkpoint()
                try:
                    super().return_next(complete_carton=True)
                except Exception:
                    self.restore(drained)
                    raise
                self.save_checkpoint('CARTON_COMPLETE')
            result = self.result(complete=True)
            result['validation'] = validate_cycle(result, self.snapshot)
            result.pop('fingerprint')
            result['fingerprint'] = digest(result)
            return result
        except ValueError as error:
            raise RuntimeError('BATCH_CONTINUATION_FAILED:'+str(error)) from error

    def result(self, *, complete=False):
        result = super().result(complete=complete)
        result.update(completion_goal=ALL_CARTONS, inventory_ids=list(self.original_boxes),
            process_history=deepcopy(self.process_history), carton_budgets=self.carton_budgets,
            batch_deadline_elapsed_s=getattr(self,'batch_deadline',self.deadline)-self.started)
        result.pop('fingerprint')
        result['fingerprint'] = digest(result)
        return result

    def save_checkpoint(self, label):
        if not self.progress_path:
            return
        # Full states contain CAD solids. Keep one lossless compressed current
        # prefix and small per-carton receipts instead of many GB of copies.
        folder = self.progress_path.parent
        result = self.result()
        temp = folder/'latest_plan.tmp.gz'
        with gzip.open(temp, 'wt', encoding='utf-8', compresslevel=3) as stream:
            stream.write(json.dumps(result, separators=(',', ':'), allow_nan=False))
        temp.replace(folder/'latest_plan.json.gz')
        save_json(folder/'batch_progress.json', dict(label=label,
            completed_ids=self.completed_ids, target_id=self.target['id'],
            q=self.current_q.tolist(), height_m=self.height_m,
            attached_id=self.target['id'] if self.attachment is not None else None,
            records=len(self.cycle_records), planning_wall_s=result['planning_wall_s'],
            physical_execution='NOT_RUN', execution_ready=False))
        if label in ('FIRST_CARTON_COMPLETE','CARTON_COMPLETE'):
            save_json(folder/f'carton_{len(self.completed_ids):02d}_complete.json',
                dict(completed_ids=self.completed_ids, process=self.process_history[-1],
                     terminal_state_fingerprint=result['terminal_state']['fingerprint'],
                     parent_stage_id=self.parent, planning_wall_s=result['planning_wall_s']))
