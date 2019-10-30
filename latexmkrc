# vim: ft=perl
my $os = `uname`;

if ( $os =~ /linux/i ) {
  $pdf_previewer = 'start okular';
} else {
  $pdf_previewer = "open -a /usr/local/Cellar/okular/19.08.2/bin/okular.app";
}
