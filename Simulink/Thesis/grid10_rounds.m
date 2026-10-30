function grid10_rounds(r0, r1, py)
% grid10_rounds.m -- the 10 ns check of the 25 ns map (grid10_check.py) in rounds without a loop turn in between, as
% r13_rounds.m does for the map. Round 1 runs out/g10_points_r1.csv (from `grid10_check.py make`; an error if it is
% missing, which is how a queued job is stopped); every later round first runs `grid10_check.py next`, which writes
% out/g10_points_r<rnd>.csv with more D0 levels for the configurations whose 10 ns powers do not bracket |P| yet, and
% stops at the first round with nothing to run. verify_steady_fine_batch: 6 runs per chunk (a 10 ns run logs 2.5 x the
% samples of a 25 ns one), 20 ms. Restart-safe: a chunk whose output exists is skipped.
%   python automation/job.py submit g10 --interactive --after r11r2 -- matlab -batch "cd('Simulink/Thesis'); grid10_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('g10_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'grid10_check.py')));
            fprintf('%s', msg);
            if st ~= 0, error('grid10_rounds: grid10_check.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('grid10_rounds: %s not found (run `grid10_check.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('grid10_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('g10_sweep_r%d.csv', rnd)), 6, 0.020);
    end
    fprintf('grid10_rounds: stopped after round %d %s\n', r1, datestr(now));
end
