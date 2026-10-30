function rev25_rounds(r0, r1, py)
% rev25_rounds.m -- the LV -> HV direction on the 25 ns grid (rev25.py) in rounds without a loop turn in between, as
% r13_rounds.m does for the map. Round 1 runs out/rev25_points_r1.csv (from `rev25.py make`; an error if it is missing, which
% is how a queued job is stopped); every later round first runs `rev25.py next`, which writes out/rev25_points_r<rnd>.csv
% with more D0 levels for the configurations whose powers do not bracket |P| yet, and stops at the first round with nothing
% to run. verify_steady_fine_batch: 12 runs per chunk, 20 ms; restart-safe.
%   python automation/job.py submit rev25 --interactive --after v1x -- matlab -batch "cd('Simulink/Thesis'); rev25_rounds(1, 4, '<python.exe>')"
    if nargin < 1 || isempty(r0), r0 = 1; end
    if nargin < 2 || isempty(r1), r1 = 4; end
    if nargin < 3 || isempty(py), py = 'python'; end
    here = fileparts(mfilename('fullpath'));
    out = fullfile(here, 'out');
    for rnd = r0:r1
        pts = fullfile(out, sprintf('rev25_points_r%d.csv', rnd));
        if rnd > 1
            [st, msg] = system(sprintf('"%s" -I -X utf8 "%s" next', py, fullfile(here, 'rev25.py')));
            fprintf('%s', msg);
            if st ~= 0, error('rev25_rounds: rev25.py next failed (exit %d)', st); end
        elseif ~isfile(pts)
            error('rev25_rounds: %s not found (run `rev25.py make` first)', pts);
        end
        if ~isfile(pts)
            fprintf('rev25_rounds: round %d has nothing to run, done %s\n', rnd, datestr(now));
            return;
        end
        verify_steady_fine_batch(pts, fullfile(out, sprintf('rev25_sweep_r%d.csv', rnd)), 12, 0.020);
    end
    fprintf('rev25_rounds: stopped after round %d %s\n', r1, datestr(now));
end
