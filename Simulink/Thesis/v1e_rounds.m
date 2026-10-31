function v1e_rounds(r0, r1, py)
% v1e_rounds.m -- V1 = 380 V (1 + e), |e| = 1, 2 % (v1e_check.py) in rounds without a loop turn in between, as v1x_rounds.m.
% Round 1 runs out/v1e_points_r1.csv (from `v1e_check.py make`; an error if it is missing, which is how a queued job is
% stopped); every later round first runs `v1e_check.py next`, which writes out/v1e_points_r<rnd>.csv with more D0 levels for
% the cases whose powers do not bracket |P| yet, and stops at the first round with nothing to run.
% verify_steady_fine_param: 12 runs per chunk, 20 ms; a chunk whose output exists is skipped.
%   python automation/job.py submit v1e --interactive --after g12a -- matlab -batch "cd('Simulink/Thesis'); v1e_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    chunk = 12;
    for rnd = r0:r1
        pts = fullfile(out, sprintf('v1e_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'v1e_check.py')));
            fprintf('%s', msg);
            if st ~= 0, error('v1e_rounds: v1e_check.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('v1e_rounds: %s not found (run `v1e_check.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('v1e_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        T = readtable(pts, 'TextType', 'string');
        N = height(T);
        for s = 1:chunk:N
            e = min(N, s + chunk - 1);
            pin = fullfile(out, sprintf('v1e_sweep_r%d_part%03d_in.csv', rnd, s));
            pout = fullfile(out, sprintf('v1e_sweep_r%d_part%03d.csv', rnd, s));
            if isfile(pout)
                fprintf('v1e_rounds: runs %d-%d of %d already in %s, skipped\n', s, e, N, pout);
                continue;
            end
            writetable(T(s:e, :), pin);
            fprintf('v1e_rounds: runs %d-%d of %d -> %s (%s)\n', s, e, N, pout, char(datetime('now')));
            verify_steady_fine_param(pin, pout, 0.020);
            delete(pin);
        end
    end
    fprintf('v1e_rounds: stopped after round %d %s\n', r1, datestr(now));
end
