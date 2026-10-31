function rev6_rounds(r0, r1, py)
% rev6_rounds.m -- the LV -> HV goal 3 points of rev6_check.py at 6.25 ns in rounds without a loop turn in between, as
% h12_rounds.m does for h12. Round 1 runs out/rev6_points_r1.csv (from `rev6_check.py make`; an error if it is missing, which
% is how a queued job is stopped); every later round first runs `rev6_check.py next`, which writes out/rev6_points_r<rnd>.csv with
% more D0 levels for the configurations that do not bracket |P| yet, and stops at the first round with nothing to run.
% verify_steady_fine_batch: 6 runs per chunk, 20 ms. Restart-safe: a chunk whose output exists is skipped.
%   python automation/job.py submit rev6 --interactive --after tgt25b -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); rev6_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('rev6_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'rev6_check.py')));
            fprintf('%s', msg);
            if st ~= 0, error('rev6_rounds: rev6_check.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('rev6_rounds: %s not found (run `rev6_check.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('rev6_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('rev6_sweep_r%d.csv', rnd)), 6, 0.020);
    end
    fprintf('rev6_rounds: stopped after round %d %s\n', r1, datestr(now));
end
