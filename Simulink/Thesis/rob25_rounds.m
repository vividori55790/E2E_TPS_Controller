function rob25_rounds(r0, r1, py)
% rob25_rounds.m -- R10 on the 25 ns grid (robust25.py) in rounds without a loop turn in between, as r13_rounds.m does for
% the map. Round 1 runs out/rob25_points_r1.csv (from `robust25.py make`; an error if it is missing, which is how a
% queued job is stopped); every later round first runs `robust25.py next`, which writes out/rob25_points_r<rnd>.csv with
% more D0 levels for the cases whose powers do not bracket |P| yet, and stops at the first round with nothing to run.
% verify_steady_fine_param: 12 runs per chunk, 20 ms; a chunk whose output exists is skipped (restart-safe).
%   python automation/job.py submit rob25 --interactive --after g10 -- matlab -batch "cd('Simulink/Thesis'); rob25_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    chunk = 12;
    for rnd = r0:r1
        pts = fullfile(out, sprintf('rob25_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'robust25.py')));
            fprintf('%s', msg);
            if st ~= 0, error('rob25_rounds: robust25.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('rob25_rounds: %s not found (run `robust25.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('rob25_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        T = readtable(pts, 'TextType', 'string');
        N = height(T);
        for s = 1:chunk:N
            e = min(N, s + chunk - 1);
            pin = fullfile(out, sprintf('rob25_sweep_r%d_part%03d_in.csv', rnd, s));
            pout = fullfile(out, sprintf('rob25_sweep_r%d_part%03d.csv', rnd, s));
            if isfile(pout)
                fprintf('rob25_rounds: runs %d-%d of %d already in %s, skipped\n', s, e, N, pout);
                continue;
            end
            writetable(T(s:e, :), pin);
            fprintf('rob25_rounds: runs %d-%d of %d -> %s (%s)\n', s, e, N, pout, char(datetime('now')));
            verify_steady_fine_param(pin, pout, 0.020);
            delete(pin);
        end
    end
    fprintf('rob25_rounds: stopped after round %d %s\n', r1, datestr(now));
end
