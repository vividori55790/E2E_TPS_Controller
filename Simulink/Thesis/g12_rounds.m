function g12_rounds(r0, r1, py)
% g12_rounds.m -- the mechanism check and the 12.5 ns grid of g12_check.py, in one job without a loop turn in between.
% First out/dg10_points.csv (10 ns, driver delay / diode transit time varied; verify_steady_fine_tp, 6 runs per chunk), then
% rounds as grid10_rounds.m: round 1 runs out/g12_points_r1.csv (from `g12_check.py make`; an error if it is missing, which is
% how a queued job is stopped), every later round first runs `g12_check.py next` and stops at the first round with nothing to
% run. verify_steady_fine_batch: 8 runs per chunk, 20 ms. Restart-safe: a chunk whose output exists is skipped.
%   python automation/job.py submit g12 --interactive --after md1 -- matlab -batch "cd('Simulink/Thesis'); g12_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    if ~isfile(fullfile(out, 'g12_points_r1.csv'))
        error('g12_rounds: out/g12_points_r1.csv not found (run `g12_check.py make` first)');
    end
    A = fullfile(out, 'dg10_points.csv');
    if ~isfile(A), error('g12_rounds: %s not found (run `g12_check.py make` first)', A); end
    T = readtable(A, 'TextType', 'string');
    N = height(T);
    chunk = 6;
    for s = 1:chunk:N
        e = min(N, s + chunk - 1);
        pin = fullfile(out, sprintf('dg10_sweep_part%03d_in.csv', s));
        pout = fullfile(out, sprintf('dg10_sweep_part%03d.csv', s));
        if isfile(pout)
            fprintf('g12_rounds: runs %d-%d of %d already in %s, skipped\n', s, e, N, pout);
            continue;
        end
        writetable(T(s:e, :), pin);
        fprintf('g12_rounds: runs %d-%d of %d -> %s (%s)\n', s, e, N, pout, char(datetime('now')));
        verify_steady_fine_tp(pin, pout, 0.020);
        delete(pin);
    end
    for rnd = r0:r1
        pts = fullfile(out, sprintf('g12_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'g12_check.py')));
            fprintf('%s', msg);
            if st ~= 0, error('g12_rounds: g12_check.py next failed (exit %d)', st); end
        end
        if ~isfile(pts)
            fprintf('g12_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('g12_sweep_r%d.csv', rnd)), 8, 0.020);
    end
    fprintf('g12_rounds: stopped after round %d %s\n', r1, datestr(now));
end
