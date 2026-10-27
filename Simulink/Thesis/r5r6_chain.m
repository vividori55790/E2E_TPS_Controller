function r5r6_chain()
% r5r6_chain.m -- R5 + R6 in ONE MATLAB session (one pool, run one after another), each step skipped if its output
% already exists, so the job can simply be resubmitted after an interruption:
%   1. staircase_check interp = 0           -> out/staircase_check_interp0.csv   (5 runs, 44 ms)
%   2. steady state at the same D0 labels   -> out/stair_steady_part*.csv       (65 runs, constant D0, 30 ms)
%   3. gen_dataset_fixed (R6, restart-safe) -> out/dataset_fixed_part_*.csv     (363 runs, 44 ms)
% Written 2026-10-27 after stair_i0 / R6_smoke stalled at 0 % CPU (MathWorks Service Host restarted at 19:11 and
% opened its window; online licensing presumably waits for a sign-in in the desktop session).
%   python automation/job.py submit r5r6 --interactive -- matlab -batch "cd('Simulink/Thesis'); r5r6_chain"
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    fprintf('r5r6_chain: start %s\n', datestr(now));
    startPoolChain(6);

    f0 = fullfile(out, 'staircase_check_interp0.csv');
    if ~isfile(f0)
        fprintf('--- step 1: staircase_check interp = 0\n');
        staircase_check('', f0, 0);
    else
        fprintf('--- step 1: %s exists, skipped\n', f0);
    end

    parts = dir(fullfile(out, 'stair_steady_part*.csv'));
    parts = parts(~endsWith({parts.name}, '_in.csv'));
    if isempty(parts)
        fprintf('--- step 2: steady state at the staircase D0 labels\n');
        verify_steady_batch(fullfile(out, 'stair_steady_points.csv'), fullfile(out, 'stair_steady.csv'), 36, 0.030);
    else
        fprintf('--- step 2: %d stair_steady part file(s) exist, skipped\n', numel(parts));
    end

    fprintf('--- step 3: gen_dataset_fixed (R6)\n');
    gen_dataset_fixed(fullfile(out, 'dataset_fixed'), [1, 9, 20], [1:2:19, 20], 18, 6);
    fprintf('r5r6_chain: done %s\n', datestr(now));
end

function startPoolChain(n)
    p = gcp('nocreate');
    if ~isempty(p) && p.Connected, return; end
    try
        c = parcluster('Processes');
        if c.NumWorkers < n, c.NumWorkers = n; end
        parpool(c, n);
        fprintf('pool: %d workers\n', n);
    catch ME
        fprintf('parpool failed: %s (the steps start their own pool or run serially)\n', ME.message);
    end
end
