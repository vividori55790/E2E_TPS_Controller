function f1eff_rounds(r0, r1, py)
% f1eff_rounds.m -- the efficiency runs of f1_eff.py in rounds without a loop turn in between, as rev6_rounds.m. Round 1 runs out/f1eff_points_r1.csv (from
% `f1_eff.py make`; an error if it is missing, which is how a queued job is stopped); every later round first runs `f1_eff.py next` and stops at the first
% round with nothing to run. verify_steady_eff_batch: 6 runs per chunk, 20 ms. Restart-safe: a chunk whose output exists is skipped.
%   python automation/job.py submit f1eff --interactive --after f1v3 -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); f1eff_rounds(1, 3, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 3; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('f1eff_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'f1_eff.py')));
            fprintf('%s', msg);
            if st ~= 0, error('f1eff_rounds: f1_eff.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('f1eff_rounds: %s not found (run `f1_eff.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('f1eff_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_eff_batch(pts, fullfile(out, sprintf('f1eff_sweep_r%d.csv', rnd)), 6, 0.020);
    end
    fprintf('f1eff_rounds: stopped after round %d %s\n', r1, datestr(now));
end
