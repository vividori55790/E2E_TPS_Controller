function h12_rounds(r0, r1, py)
% h12_rounds.m -- the 98.7 V slice of h12_check.py at 12.5 ns and 10 ns in rounds without a loop turn in between, as
% grid10_rounds.m does for g10. Round 1 runs out/h12_points_r1.csv (from `h12_check.py make`; an error if it is missing, which
% is how a queued job is stopped); every later round first runs `h12_check.py next`, which writes out/h12_points_r<rnd>.csv with
% more D0 levels for the configurations that do not bracket |P| yet, and stops at the first round with nothing to run.
% verify_steady_fine_batch: 6 runs per chunk, 20 ms. Restart-safe: a chunk whose output exists is skipped.
%   python automation/job.py submit h12 --interactive --after edge25 -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); h12_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('h12_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'h12_check.py')));
            fprintf('%s', msg);
            if st ~= 0, error('h12_rounds: h12_check.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('h12_rounds: %s not found (run `h12_check.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('h12_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('h12_sweep_r%d.csv', rnd)), 6, 0.020);
    end
    fprintf('h12_rounds: stopped after round %d %s\n', r1, datestr(now));
end
