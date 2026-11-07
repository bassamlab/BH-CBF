# BH-CBF

Project page of the paper "Safeguarding Multi-Agent Reinforcement Learning for Connected and Automated Vehicles with Recursively Feasible Control Barrier Functions" (BH-CBF).

- Paper: https://arxiv.org/abs/2611.04957
- Project page: https://bassamlab.github.io/BH-CBF
- Code: https://github.com/bassamlab/SigmaRL/tree/1.7.0 (tag 1.7.0)

## Interactive demo

`assets/widget/` holds the interactive demo of the section "Try It Yourself": `bhcbf.js` (the single-vehicle BH-CBF of the paper), `sim.js` (closed loop), `widget.js` and `widget.css` (user interface), and `roundabout.json` (road and route). The data and the reference values come from `sigmarl/evaluation_tits26/video_demo/widget_data.py` of SigmaRL; `node assets/widget/check_model.mjs <widget_reference.json>` compares `bhcbf.js` with the Python implementation.
