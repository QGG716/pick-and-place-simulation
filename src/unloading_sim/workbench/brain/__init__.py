"""Operator assistant ("brain") for the ECO65 workbench (user request 2026-11-28).

A Qwen2.5-VL vision-language model on the remote GPU server looks at the live
colour frame and (1) says whether pick-and-place may start and whether the
carton stack shows a collapse risk, and (2) answers the operator, who talks to
it through local CPU speech recognition and speech synthesis.

User decisions of 2026-11-28: the VLM is ADVISORY ONLY - its verdict is shown
and spoken but never gates, authorizes or alters a plan, a cycle or a job;
ASR and TTS run on the field PC CPU; voice may ask questions and may say
"停止", which triggers the same software stop as the stop button (a stop never
releases a held load and never replaces the hardware emergency stop); voice
can never start a job or move the robot.
"""
AUTHORITY = 'ADVISORY_ONLY_NO_GATE'
