# vim: ft=perl
my $os = `uname`;

if ( $os =~ /linux/i ) {
  $pdf_previewer = 'start okular';
} else {
  # brew tap kde-mac/kde
  # brew install cpanminus
  # cpanm URI
  # brew install okular
  $pdf_previewer = "open -a /usr/local/Cellar/okular/19.08.2/bin/okular.app";
}
