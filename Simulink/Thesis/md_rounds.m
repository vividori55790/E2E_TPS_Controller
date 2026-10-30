function md_rounds(r0, r1, py)
% md_rounds.m -- R13 stage 2 (md_check.py): the 25 ns verification of the retrained surface M-D in rounds without a loop
% turn in between, as r13_rounds.m does for the map. Round 1 runs out/md_points_r1.csv (from `md_check.py make`; an error if
% it is missing, which is how a queued job is stopped); every later round first runs `md_check.py next`, which writes
% out/md_points_r<rnd>.csv with more D0 levels for the configurations whose powers do not bracket |P| yet, and stops at the
% first round with nothing to run. verify_steady_fine_batch: 12 runs per chunk, 20 ms. Restart-safe: a chunk whose output
% exists is skipped, and `next` only lists what is still unbracketed.
%   python automation/job.py submit md1 --interactive --after rob25 -- matlab -batch "cd('Simulink/Thesis'); md_rounds(1, 5, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 5; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('md_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'md_check.py')));
            fprintf('%s', msg);
            if st ~= 0, error('md_rounds: md_check.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('md_rounds: %s not found (run `md_check.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('md_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('md_sweep_r%d.csv', rnd)), 12, 0.020);
    end
    fprintf('md_rounds: stopped after round %d %s\n', r1, datestr(now));
end
