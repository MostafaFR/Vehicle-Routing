$(function () {
  var trackDownloads = function (event) {
    var s=s_gi(s_account);
    s.linkTrackVars='events';
    s.linkTrackEvents='event45';
    s.events='event45';
    s.tl(this,'o','fileexdwl');
  };
  $('[href$="download=true"]').on('click', trackDownloads);
});
