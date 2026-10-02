"""Scoped adapter for the pinned SAM entry: one image encoder per request.

The original upstream perturbation, decoder, medoid and NMS remain unchanged.
No model patch or feature survives the context. No external checkout is edited.
"""
from contextlib import contextmanager
from copy import copy,deepcopy
from time import perf_counter


@contextmanager
def sam_image_session(entry,processor,model,torch,*,identity,reuse=True):
    report=dict(identity=identity,reuse_image_embedding=reuse,image_preprocess_calls=0,
        image_encoder_calls=0,prompt_decoder_calls=0,image_encoding_seconds=0.,
        model_forward_seconds=0.,image_preprocess_seconds=0.,actual_prompts=[])
    original_image_processor=processor.image_processor
    cached_encoding=None;seen_image=None;embedding=None
    class Images:
        def __getattr__(self,name):return getattr(original_image_processor,name)
        def __call__(self,image,*args,**kwargs):
            nonlocal cached_encoding,seen_image
            if seen_image is not None and image is not seen_image:raise ValueError('SAM_IMAGE_CHANGED_WITHIN_REQUEST')
            seen_image=image
            if cached_encoding is None or not reuse:
                started=perf_counter();result=original_image_processor(image,*args,**kwargs)
                report['image_preprocess_seconds']+=perf_counter()-started;report['image_preprocess_calls']+=1
                if reuse:cached_encoding=deepcopy(result)
                return result
            return deepcopy(cached_encoding)
    scoped_processor=copy(processor);scoped_processor.image_processor=Images()
    class Model:
        def __getattr__(self,name):return getattr(model,name)
        def to(self,*a,**kw):return self
        def eval(self):return self
        def __call__(self,**inputs):
            nonlocal embedding
            torch.cuda.synchronize();start=perf_counter()
            if reuse:
                if embedding is None:
                    embedding=model.get_image_embeddings(inputs['pixel_values'])
                    torch.cuda.synchronize();report['image_encoding_seconds']+=perf_counter()-start
                    report['image_encoder_calls']+=1
                inputs=dict(inputs);inputs.pop('pixel_values');inputs['image_embeddings']=embedding
                start=perf_counter()
            else:report['image_encoder_calls']+=1
            result=model(**inputs);torch.cuda.synchronize()
            report['model_forward_seconds']+=perf_counter()-start;report['prompt_decoder_calls']+=1
            return result
    original=entry.perturb_boxes
    def perturb(box,width,height):
        variants=original(box,width,height)
        report['actual_prompts'].append(dict(base_bbox=list(box),image_size=[width,height],variants=deepcopy(variants),
            source='ACTUAL_PINNED_PERTURB_BOXES_RETURN_BEFORE_PROCESSOR'))
        return variants
    entry.perturb_boxes=perturb
    try:yield scoped_processor,Model(),report
    finally:entry.perturb_boxes=original;embedding=None;cached_encoding=None
