function r5r6_chain()
% r5r6_chain.m -- R5 + R6 in ONE MATLAB session (one pool, run one after another), each step skipped if its output
% already exists, so the job can simply be resubmitted after an interruption:
%   1. staircase_check interp = 0           -> out/staircase_check_interp0.csv   (5 runs, 44 ms)
%   2. steady state at the same D0 labels   -> out/stair_steady_part*.csv       (106 runs: 65 staircase labels + 18 K1_
%                                                                                 + 23 BT_, constant D0, 30 ms)
%  2b. efficiency runs (R8, 10-29)          -> out/eff_sweep_r1_part*.csv       (38 runs, 30 ms)
%   3. gen_dataset_fixed (R6, restart-safe) -> out/dataset_fixed_part_*.csv     (363 runs, 44 ms)
% Written 2026-10-27 after stair_i0 / R6_smoke stalled at 0 % CPU (MathWorks Service Host restarted at 19:11 and
% opened its window; online licensing presumably waits for a sign-in in the desktop session). 10-28: first waits
% (<= 40 min) for those two jobs' .done files, since all three resume at the same sign-in. 10-28 16:50: an error in
% one step no longer cancels the later ones (step 2 needs neither step 1 nor the D0 block change); the job still
% ends with an error if any step failed, so job.py reports it.
%   python automation/job.py submit r5r6 --interactive -- matlab -batch "cd('Simulink/Thesis'); r5r6_chain"
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    fprintf('r5r6_chain: start %s\n', datestr(now));
    % stair_i0 (= step 1, serial) and R6_smoke (own 6-worker pool) stalled at the same sign-in and resume with this
    % session: let them finish first -- one pool at a time on 6 cores / 31 GB, and step 1 is then already on disk
    waitForJobs(fullfile(fileparts(fileparts(here)), 'automation', 'state', 'jobs'), {'stair_i0', 'R6_smoke'}, 40);
    startPoolChain(6);
    failed = {};

    f0 = fullfile(out, 'staircase_check_interp0.csv');
    if ~isfile(f0)
        fprintf('--- step 1: staircase_check interp = 0\n');
        failed = runStep(failed, 'step 1', @() staircase_check('', f0, 0));
    else
        fprintf('--- step 1: %s exists, skipped\n', f0);
    end

    % 10-29: always called -- verify_steady_batch skips the chunks whose part file exists (a reboot at 19:05 stopped
    % this step inside its first chunk; before, any existing part file skipped the whole step)
    fprintf('--- step 2: steady state at the staircase D0 labels\n');
    failed = runStep(failed, 'step 2', @() verify_steady_batch(fullfile(out, 'stair_steady_points.csv'), ...
                                                            fullfile(out, 'stair_steady.csv'), 36, 0.030));

    % 10-29 (R8, user list): efficiency curve -- the 200/400/800 W configurations R.4 did not run (38 runs x 30 ms,
    % ~25 min; efficiency_sweep.py next). Before step 3 because it is short; restart-safe per 24-run chunk.
    fe = fullfile(out, 'eff_points_r1.csv');
    if isfile(fe)
        fprintf('--- step 2b: efficiency runs (R8)\n');
        failed = runStep(failed, 'step 2b', @() verify_steady_batch(fe, fullfile(out, 'eff_sweep_r1.csv'), 24, 0.030));
    end

    fprintf('--- step 3: gen_dataset_fixed (R6)\n');
    failed = runStep(failed, 'step 3', @() gen_dataset_fixed(fullfile(out, 'dataset_fixed'), [1, 9, 20], ...
                                                            [1:2:19, 20], 18, 6));
    fprintf('r5r6_chain: done %s\n', datestr(now));
    if ~isempty(failed), error('r5r6_chain: %s failed (reports above)', strjoin(failed, ', ')); end
end

function failed = runStep(failed, name, fn)
% run one step; on an error print the full report and go on (the later steps do not read its output)
    try
        fn();
    catch ME
        fprintf('%s FAILED: %s\n', name, getReport(ME, 'extended', 'hyperlinks', 'off'));
        failed{end + 1} = name;
    end
end

function waitForJobs(jobsDir, names, maxMin)
% job.py writes <name>.done when a job's MATLAB exits (success or not); give up after maxMin minutes
    t0 = tic; lastMsg = -inf;
    while true
        pending = names(~cellfun(@(nm) isfile(fullfile(jobsDir, [nm '.done'])), names));
        if isempty(pending)
            fprintf('other MATLAB jobs finished (waited %.0f s)\n', toc(t0));
            return;
        end
        if toc(t0) > maxMin * 60
            fprintf('still running after %d min: %s -- starting anyway\n', maxMin, strjoin(pending, ', '));
            return;
        end
        if toc(t0) - lastMsg >= 300
            fprintf('[%s] waiting for %s to finish\n', datestr(now), strjoin(pending, ', '));
            lastMsg = toc(t0);
        end
        pause(30);
    end
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
